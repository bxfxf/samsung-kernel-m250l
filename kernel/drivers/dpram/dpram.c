/* dpram.c
 *
 * SAMSUNG TTY modem driver for cdma
 *
 * Copyright (C) 2010 Samsung Electronics. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define _HSDPA_DPRAM
#undef _ENABLE_ERROR_DEVICE

#define _ENABLE_DEBUG_PRINTS

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <mach/hardware.h>

#include <mach/hardware.h>
#include <mach/map.h>
#include <mach/regs-mem.h>
#include <mach/gpio.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/wakelock.h>

#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/ip.h>
#include <linux/icmp.h>
#include <linux/time.h>
#include <linux/if_arp.h>

/* TODO: kthur
   add debug, : ramdump mode
   remove useless log
 */
#include "dpram.h"

#define SVNET_PDP_ETHER
#ifdef SVNET_PDP_ETHER
#include <linux/if_ether.h>
#include <linux/skbuff.h>
#endif

#define DRIVER_NAME 		"DPRAM"
#define DRIVER_PROC_ENTRY	"driver/dpram"
#define DRIVER_MAJOR_NUM	252

#ifdef _DEBUG
#define dprintk(s, args...) printk("[DPRAM] %s:%d - " s, __func__, __LINE__,  ##args)
#else
#define dprintk(s, args...)
#endif	/* _DEBUG */


#define WRITE_TO_DPRAM(dest, src, size) \
	_memcpy((void *)(DPRAM_VBASE + dest), src, size)

#define READ_FROM_DPRAM(dest, src, size) \
	_memcpy(dest, (void *)(DPRAM_VBASE + src), size)


#undef LOOP_BACK_TEST			/* Use Loopback test via CH.31 */
#undef LOCAL_LOOP_BACK_TEST			/* Use Loopback test via CH.31 */

#define USE_INTERRUPTABLE_LOAD
#undef ENABLE_INTERRUPTABLE_UPLOAD

#define USE_WAIT_4COMPLETE

/*****************************************************************************/
/*                             MULTIPDP FEATURE                              */
/*****************************************************************************/
#include <linux/miscdevice.h>
#include <linux/netdevice.h>

/* Device node name for application interface */
#define APP_DEVNAME				"multipdp"
/* number of PDP context */
#define NUM_PDP_CONTEXT			14

/* Device types */
#define DEV_TYPE_NET			0 /* network device for IP data */
#define DEV_TYPE_SERIAL			1 /* serial device for CSD */

/* Device major & minor number */
#define CSD_MAJOR_NUM			251
#define CSD_MINOR_NUM			0

/* Maximum number of PDP context */
#define MAX_PDP_CONTEXT			14

/* Maximum PDP data length */
#define MAX_PDP_DATA_LEN		1500

#define DPRAM_TX_RETRY               2000

/* Device flags */
#define DEV_FLAG_STICKY			0x1 /* Sticky */

/* Device Identification */
#define ONEDRAM_DEV 			0
#define DPRAM_DEV			1
#define UNKNOWN_DEV			2

/* Radio Tye Identification */
#define LTE			    0
#define CDMA			1

/* Data Status */
#define SUSPEND			0
#define ACTIVE			1

/* Maximum PDP packet length including header and start/stop bytes */
#define MAX_PDP_PACKET_LEN		(MAX_PDP_DATA_LEN + 4 + 2)

typedef enum _interface_type
{
	IPADDR_TYPE_IPV4 = 1,
	IPADDR_TYPE_IPV6,
	IPADDR_TYPE_IPV4V6,
	IPV6_TYPE_MAX
}interface_type_t;

typedef struct pdp_arg {
	unsigned char	id;
	char		ifname[16];
	char 		int_iden[8];
	interface_type_t en_interface_type;
} __attribute__ ((packed)) pdp_arg_t;


/* PDP data packet header format */
struct pdp_hdr {
	u16	len;		/* Data length */
	u8	id;			/* Channel ID */
	u8	control;	/* Control field */
} __attribute__ ((packed));

/* Virtual network interface */
struct vnet_struct{
	struct net_device	*net;
	struct net_device_stats	stats;
	struct work_struct	xmit_work_struct;
	struct sk_buff* 	skb_ptr;
	u8	    netq_init;				//0:netif_start_queue called    1: netif_start_queue() not called
	u8	    netq_active;			//0: don't call netif_wake_queue    1: Call netif_wake_queue
	struct semaphore netq_sem;	// sem to protect netq_init and netq_active
};

/* PDP information type */
struct pdp_info {
	/* PDP context ID */
	u8		id;
	/* onedram or dpram interface */
	atomic_t		intf_dev;

	/* Device type */
	unsigned		type;

	/* Device flags */
	unsigned		flags;

	/* Tx packet buffer */
	u8		*tx_buf;

	/* App device interface */
	union {
		/* Virtual network interface */
		struct vnet_struct vnet_u;

		/* Virtual serial interface */
		struct {
			struct tty_driver	tty_driver[NUM_PDP_CONTEXT];
			int         		refcount;
			struct tty_struct	*tty_table[1];
			struct ktermios		*termios[1];
			struct ktermios		*termios_locked[1];
			char			tty_name[16];
			struct tty_struct	*tty;
			struct semaphore	write_lock;
		} vs_u;
	} dev_u;
#define vn_dev		dev_u.vnet_u
#define vs_dev		dev_u.vs_u
};

/*
------------------
Buffer : 31KByte
------------------
Reserved: 1014Byte
------------------
SIZE: 2Byte
------------------
TAG: 2Byte
------------------
COUNT: 2Byte
------------------
AP -> CP Intr : 2Byte
------------------
CP -> AP Intr : 2Byte
------------------
*/
#define DPRAM_DLOAD_START_ADDRESS 					0x0000					
#define DPRAM_DLOAD_BUFF_ADDRESS					(DPRAM_START_ADDRESS)
#define DPRAM_DLOAD_RESV_ADDRESS					(DPRAM_START_ADDRESS + 0x7BFE)
#define DPRAM_DLOAD_SIZE_ADDRESS						(DPRAM_START_ADDRESS + 0x7FF6)
#define DPRAM_DLOAD_TAG_ADDRESS						(DPRAM_START_ADDRESS + 0x7FF8)
#define DPRAM_DLOAD_COUNT_ADDRESS					(DPRAM_START_ADDRESS + 0x7FFA)

#if defined(USE_INTERRUPTABLE_LOAD)
struct _param_test {
	unsigned int total_size;
	unsigned int rest_size;
	unsigned int send_size;
	unsigned int copy_start;
	unsigned int copy_complete;
	unsigned int boot_complete;
#if defined(ENABLE_INTERRUPTABLE_UPLOAD)
	unsigned int upload_copy_start;
	unsigned int upload_copy_complete;
#endif /* ENABLE_INTERRUPTABLE_UPLOAD */

	struct work_struct work_directly;
	
};

static struct _param_nv *data_param;
static struct _param_test check_param;

unsigned char *img=NULL;
#endif

#if defined(USE_WAIT_4COMPLETE)
unsigned int boot_start_complete=0;
#endif /* USE_WAIT_4COMPLETE */


/* PDP information table */
static struct pdp_info *pdp_table[MAX_PDP_CONTEXT];
static DECLARE_MUTEX(pdp_lock);

static int g_adjust = 9;
static int g_datastatus = ACTIVE;//To Suspend or resume data
static int pdp_tx_flag = 0;

static unsigned char pdp_net_rx_buf[MAX_PDP_DATA_LEN + 100];

static inline struct pdp_info *pdp_get_dev(u8 id);
static int dpram_init_and_report(void);
void ClearPendingInterruptFromModem(void);

static int pdp_mux(struct pdp_info *dev, const void *data, size_t len);
static inline struct pdp_info * pdp_get_serdev(const char *name);
static inline struct pdp_info *pdp_remove_dev(u8 id);
static int  vnet_recv_rx_data(u8 *buf, int size, struct pdp_info *dev);
void dpram_debug_dump_raw_read_buffer(const unsigned char  *buf, int len);
void dpram_debug_dump_write_buffer(const unsigned char  *buf, int len);
void multipdp_debug_dump_write_buffer(const unsigned char  *buf, int len);

static irqreturn_t dpram_irq_handler(int irq, void *dev_id);
static void unregister_dpram_driver(void);
static int register_interrupt_handler(void);

unsigned int Gdpram_irq;


/*****************************************************************************/
#ifdef _ENABLE_DEBUG_PRINTS

#define PRINT_WRITE
#define PRINT_READ
#define PRINT_WRITE_SHORT
#define PRINT_READ_SHORT
#define PRINT_SEND_IRQ
#define PRINT_RECV_IRQ
#define PRINT_IPC_FORMATTED_MSG

#define DPRAM_PRINT_ERROR             0x0001
#define DPRAM_PRINT_WARNING           0x0002
#define DPRAM_PRINT_INFO              0x0004
#define DPRAM_PRINT_WRITE             0x0008
#define DPRAM_PRINT_WRITE_SHORT       0x0010
#define DPRAM_PRINT_READ              0x0020
#define DPRAM_PRINT_READ_SHORT        0x0040
#define DPRAM_PRINT_SEND_IRQ          0x0080
#define DPRAM_PRINT_RECV_IRQ          0x0100
#define DPRAM_PRINT_HEAD_TAIL         0x0400
#define DPRAM_PRINT_IPC_FORMATTED_MSG 0x0800
#define DPRAM_PRINT_CBUF 			0x1000

#define MULTIPDP_PRINT_ERROR		0x0001
#define MULTIPDP_PRINT_WARNING		0x0002
#define MULTIPDP_PRINT_INFO			0x0004
#define MULTIPDP_PRINT_WRITE		0x0008
#define MULTIPDP_PRINT_READ			0x0010

void dpram_debug_print(u8 print_prefix, u32 mask,  const char *fmt, ...);
void register_dpram_debug_control_attribute(void);
void deregister_dpram_debug_control_attribute(void);
static ssize_t dpram_show_debug_level(struct device_driver *ddp, char *buf);
static ssize_t dpram_store_debug_level(struct device_driver *ddp, const char *buf, size_t count);

#define DPRAM_LOG_ERR(s, arg...)           dpram_debug_print(1, DPRAM_PRINT_ERROR, s, ## arg)
#define DPRAM_LOG_WARN(s, arg...)          dpram_debug_print(1, DPRAM_PRINT_WARNING, s, ## arg)
#define DPRAM_LOG_INFO(s, arg...)          do { } while (0) //dpram_debug_print(1, DPRAM_PRINT_INFO, s, ## arg)
#define DPRAM_LOG_WRITE(s, arg...)        do { } while (0) //dpram_debug_print(1, DPRAM_PRINT_WRITE, s, ## arg)
#define DPRAM_LOG_WRITE_SHORT(s, arg...)  do { } while (0) //dpram_debug_print(1, DPRAM_PRINT_WRITE_SHORT, s, ## arg)
#define DPRAM_LOG_READ(s, arg...)         do { } while (0) //dpram_debug_print(1, DPRAM_PRINT_READ, s, ## arg)
#define DPRAM_LOG_READ_SHORT(s, arg...)   do { } while (0) //dpram_debug_print(1, DPRAM_PRINT_READ_SHORT, s, ## arg)
#define DPRAM_LOG_SEND_IRQ(s, arg...)     do { } while (0) //dpram_debug_print(1, DPRAM_PRINT_SEND_IRQ, s, ## arg)
#define DPRAM_LOG_RECV_IRQ(s, arg...)     do { } while (0) //dpram_debug_print(1, DPRAM_PRINT_RECV_IRQ, s, ## arg)
#define DPRAM_LOG_HEAD_TAIL(s, arg...)    do { } while (0) //dpram_debug_print(1, DPRAM_PRINT_HEAD_TAIL, s, ## arg)
#define DPRAM_LOG_FIPC_MSG(s, arg...)     do { } while (0) //dpram_debug_print(0, DPRAM_PRINT_IPC_FORMATTED_MSG, s, ## arg)

void multipdp_debug_print(u32 mask,  const char *fmt, ...);
void register_multipdp_debug_control_attribute(void);
void deregister_multipdp_debug_control_attribute(void);
#define MULTIPDP_LOG_ERR(s,arg...)			multipdp_debug_print(MULTIPDP_PRINT_ERROR, s, ## arg)
#define MULTIPDP_LOG_WARN(s,arg...)			multipdp_debug_print(MULTIPDP_PRINT_WARNING, s, ## arg)
#define MULTIPDP_LOG_INFO(s,arg...)			multipdp_debug_print(MULTIPDP_PRINT_INFO, s, ## arg)
#define MULTIPDP_LOG_READ(s,arg...)			multipdp_debug_print(MULTIPDP_PRINT_READ, s, ## arg)
#define MULTIPDP_LOG_WRITE(s,arg...)			multipdp_debug_print(MULTIPDP_PRINT_WRITE, s, ## arg)

u16 dpram_debug_mask = DPRAM_PRINT_ERROR | DPRAM_PRINT_WARNING;
//u16 dpram_debug_mask = DPRAM_PRINT_INFO | DPRAM_PRINT_ERROR | DPRAM_PRINT_WARNING;
//u16 dpram_debug_mask = DPRAM_PRINT_HEAD_TAIL | DPRAM_PRINT_RECV_IRQ | DPRAM_PRINT_SEND_IRQ | DPRAM_PRINT_READ | DPRAM_PRINT_WRITE | DPRAM_PRINT_ERROR | DPRAM_PRINT_WARNING | DPRAM_PRINT_IPC_FORMATTED_MSG | DPRAM_PRINT_READ_SHORT | DPRAM_PRINT_WRITE_SHORT | DPRAM_PRINT_INFO;

//u16 mulitpdp_debug_mask = MULTIPDP_PRINT_ERROR | MULTIPDP_PRINT_WARNING;
u16 mulitpdp_debug_mask = MULTIPDP_PRINT_WRITE | MULTIPDP_PRINT_READ | MULTIPDP_PRINT_ERROR | MULTIPDP_PRINT_WARNING;
//u16 mulitpdp_debug_mask = MULTIPDP_PRINT_ERROR | MULTIPDP_PRINT_WARNING | MULTIPDP_PRINT_INFO | MULTIPDP_PRINT_WRITE | MULTIPDP_PRINT_READ;

#else

#define DPRAM_LOG_ERR(s, arg...)           do { } while (0)
#define DPRAM_LOG_WARN(s, arg...)          do { } while (0)
#define DPRAM_LOG_INFO(s, arg...)          do { } while (0)
#define DPRAM_LOG_WRITE(s, arg...)        do { } while (0)
#define DPRAM_LOG_WRITE_SHORT(s, arg...)  do { } while (0)
#define DPRAM_LOG_READ(s, arg...)         do { } while (0)
#define DPRAM_LOG_READ_SHORT(s, arg...)   do { } while (0)
#define DPRAM_LOG_SEND_IRQ(s, arg...)     do { } while (0)
#define DPRAM_LOG_RECV_IRQ(s, arg...)     do { } while (0)
#define DPRAM_LOG_HEAD_TAIL(s, arg...)    do { } while (0)
#define DPRAM_LOG_FIPC_MSG(s, arg...)     do { } while (0)

#define MULTIPDP_LOG_ERR(s,arg...)			do { } while (0)
#define MULTIPDP_LOG_WARN(s,arg...)			do { } while (0)
#define MULTIPDP_LOG_INFO(s,arg...)			do { } while (0)
#define MULTIPDP_LOG_READ(s,arg...)			do { } while (0)
#define MULTIPDP_LOG_WRITE(s,arg...)		do { } while (0)

#endif /*_ENABLE_DEBUG_PRINTS */

#ifdef _ENABLE_ERROR_DEVICE
#define DPRAM_ERR_MSG_LEN			128 //65
#define DPRAM_ERR_DEVICE			"dpramerr"
#endif	/* _ENABLE_ERROR_DEVICE */


/***************************************************************************/
/*                              GPIO SETTING                               */
/***************************************************************************/
#include <mach/gpio.h>

#define GPIO_LEVEL_LOW		0
#define GPIO_LEVEL_HIGH		1

#define IRQ_DPRAM_INT_N		IRQ_EINT8 //IRQ_EINT12

static void init_hw_setting(void);
static int dpram_shared_bank_remap(void);
static void dpram_platform_init(void);
static void send_interrupt_to_phone(u16 irq_mask);

static void __iomem *dpram_base;
static void dpram_drop_data(dpram_device_t *device);

static int phone_sync;
//static int dump_on;
static int dpram_phone_getstatus(void);

#define DPRAM_VBASE dpram_base

static struct tty_driver *dpram_tty_driver;

static dpram_tasklet_data_t dpram_tasklet_data[MAX_INDEX];

static dpram_device_t dpram_table[MAX_INDEX] = {
	{
		.in_head_addr = DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS,
		.in_tail_addr = DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS,
		.in_buff_addr = DPRAM_PHONE2PDA_FORMATTED_BUFFER_ADDRESS,
		.in_buff_size = DPRAM_PHONE2PDA_FORMATTED_BUFFER_SIZE,
		.in_head_saved = 0,
		.in_tail_saved = 0,

		.out_head_addr = DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS,
		.out_tail_addr = DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS,
		.out_buff_addr = DPRAM_PDA2PHONE_FORMATTED_BUFFER_ADDRESS,
		.out_buff_size = DPRAM_PDA2PHONE_FORMATTED_BUFFER_SIZE,
		.out_head_saved = 0,
		.out_tail_saved = 0,

		.mask_req_ack = INT_MASK_REQ_ACK_F,
		.mask_res_ack = INT_MASK_RES_ACK_F,
		.mask_send = INT_MASK_SEND_F,
	},
	{
		.in_head_addr = DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS,
		.in_tail_addr = DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS,
		.in_buff_addr = DPRAM_PHONE2PDA_RAW_BUFFER_ADDRESS,
		.in_buff_size = DPRAM_PHONE2PDA_RAW_BUFFER_SIZE,
		.in_head_saved = 0,
		.in_tail_saved = 0,

		.out_head_addr = DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS,
		.out_tail_addr = DPRAM_PDA2PHONE_RAW_TAIL_ADDRESS,
		.out_buff_addr = DPRAM_PDA2PHONE_RAW_BUFFER_ADDRESS,
		.out_buff_size = DPRAM_PDA2PHONE_RAW_BUFFER_SIZE,
		.out_head_saved = 0,
		.out_tail_saved = 0,

		.mask_req_ack = INT_MASK_REQ_ACK_R,
		.mask_res_ack = INT_MASK_RES_ACK_R,
		.mask_send = INT_MASK_SEND_R,
	},
};

static struct tty_struct *dpram_tty[MAX_INDEX];
static struct ktermios *dpram_termios[MAX_INDEX];
static struct ktermios *dpram_termios_locked[MAX_INDEX];

static void res_ack_tasklet_handler(unsigned long data);
static void fmt_rcv_tasklet_handler(unsigned long data);
static void raw_rcv_tasklet_handler(unsigned long data);

static DECLARE_TASKLET(fmt_send_tasklet, fmt_rcv_tasklet_handler, 0);
static DECLARE_TASKLET(raw_send_tasklet, raw_rcv_tasklet_handler, 0);
static DECLARE_TASKLET(fmt_res_ack_tasklet, res_ack_tasklet_handler, (unsigned long)&dpram_table[FORMATTED_INDEX]);
static DECLARE_TASKLET(raw_res_ack_tasklet, res_ack_tasklet_handler, (unsigned long)&dpram_table[RAW_INDEX]);

#ifdef _ENABLE_ERROR_DEVICE
static unsigned int is_dpram_err= FALSE;
static char dpram_err_buf[DPRAM_ERR_MSG_LEN];

struct class *dpram_class;
static DECLARE_WAIT_QUEUE_HEAD(dpram_err_wait_q);
static struct fasync_struct *dpram_err_async_q;
#endif	/* _ENABLE_ERROR_DEVICE */

struct wake_lock dpram_wake_lock;
static atomic_t    dpram_write_lock;

#define PIF_TIMEOUT		180 * HZ

static atomic_t raw_txq_req_ack_rcvd;
static atomic_t fmt_txq_req_ack_rcvd;

/* if you enable this, remember there will be multi path to report the error to upper layer
   1 from /dev/dpram1 to error device and 2nd one from multipdp to error device.
   you should sync these two path */
//#define _ENABLE_SELF_ERROR_CORRECTION

void dpram_initiate_self_error_correction(void)
{
#ifdef _ENABLE_SELF_ERROR_CORRECTION
	DPRAM_LOG_ERR ("[DPRAM] dpram_initiate_self_error_correction\n");
	request_phone_reset();
#endif
}

static int modem_condition_count=0;
static int verification_condition_write=0;
static int verification_condition_read=0;

#ifdef LOOP_BACK_TEST

/**********************************************************************
	loop back test implementation		

	Usage :

	1. start test
	
	loopback test can be triggered by setting '/sys/class/misc/multipdp/loopback'

	"
	# cd /sys/class/misc/multipdp/
	# echo start > loopback
	
	2. get stastic result
	
	it shows some result value for this test)
	
	"
	# cat loopback

	3. stop test

	it stops loopback test

	"
	# echo stop > loopback
	(be careful, it does not show any result)


**********************************************************************/
#define LOOP_BACK_CHANNEL	31

int loopback_ongoing = 0;

char loopback_data[MAX_PDP_DATA_LEN];

char loopback_value[256];

struct loopback_result
{
	int nTransfered;
	int nPacketDataSize;
	struct timeval nStartTime;
	struct timeval nEndTime;
};

static struct loopback_result loopback_res;

static void test_loopback_write(int count)
{
	
	// initialize stastics value
	loopback_res.nTransfered = 1;
	loopback_res.nPacketDataSize = count;
	
	do_gettimeofday(&loopback_res.nStartTime);

	DPRAM_LOG_ERR( "Write    Size = %d    nStartTime = %d\n",
		loopback_res.nPacketDataSize,
		loopback_res.nStartTime
	);
}

static void test_loopback_read(void)
{
	unsigned int nElapsedtime_s;
	
	do_gettimeofday(&loopback_res.nEndTime);

	DPRAM_LOG_ERR( "Write    Size = %d    nEndTime = %d\n",
		loopback_res.nPacketDataSize,
		loopback_res.nEndTime
	);
}

static ssize_t show_loopback_value(struct device *dev, struct device_attribute *attr, char * buf)
{
	unsigned int nElapsedtime_s, total_size;
	if(!strncmp(loopback_value, "start", 5)) {
		// show elapsed value
		do_gettimeofday(&loopback_res.nEndTime);

		nElapsedtime_s = (loopback_res.nEndTime.tv_sec - loopback_res.nStartTime.tv_sec)
			+ (loopback_res.nEndTime.tv_usec - loopback_res.nStartTime.tv_usec)/1000000;

		total_size = loopback_res.nTransfered * loopback_res.nPacketDataSize;
		
		return sprintf(buf,
			"\n=====	LoopBack Test Result	=====\n\n"
			"Transfered Items = %d\n"
			"Packet Data Size = %d\n"
			"Total transfer size = %d\n"
			"Elapsed Time = %d (s)\n"
			"Mean Value = %d (byte/sec)\n"
			"\n=====================================\n"
			,
			loopback_res.nTransfered,
			loopback_res.nPacketDataSize,
			total_size,
			nElapsedtime_s,
			total_size/nElapsedtime_s
			);
						
	}
	else {
		// show test is not on going.
		return sprintf(buf, "loopback test is not on going\n");
	}
}

static send_loop_back_packet(const char* data, int size)
{
	struct pdp_info* dev = pdp_get_dev(LOOP_BACK_CHANNEL);

	if (loopback_ongoing) {
		//printk("send loopback packet start [%d]\n",loopback_res.nTransfered);
		pdp_mux(dev, data, size);
		//printk("send loopback packet end [%d]\n",loopback_res.nTransfered);
		loopback_res.nTransfered++;
	}
}

static ssize_t store_loopback_value(struct device *dev, struct device_attribute *attr, char * buf, size_t count)
{
	int i;

	// we can send various size of data by setting this value as mutiple of 10
	int data_size = 1500;
	
	char temp_str[10] = "0123456789";
	if ( !strncmp(buf, "start", 5)) {
		sscanf(buf, "%s", loopback_value);

		// initialize stastics value
		loopback_res.nTransfered = 0;
		loopback_res.nPacketDataSize = data_size;
		
		// make data
		for (i = 0; i < (data_size/10); i++) {
			memcpy((loopback_data + i*10), temp_str, 10);
		}

		loopback_ongoing = 1;
		
		do_gettimeofday(&loopback_res.nStartTime);
		
		send_loop_back_packet(loopback_data, data_size);
	}
	else if (!strncmp(buf, "stop", 4)) {
		sscanf(buf, "%s", loopback_value);
		
		loopback_ongoing = 0;
		
		do_gettimeofday(&loopback_res.nEndTime);
	}
	return strnlen(buf, 256);
}

static DEVICE_ATTR(loopback, S_IRUGO|S_IWUSR, show_loopback_value, store_loopback_value);
#endif


/* tty related functions. */
static inline void byte_align(unsigned long dest, unsigned long src)
{
	u16 *p_src;
	volatile u16 *p_dest;

	if (!(dest % 2) && !(src % 2)) {
		p_dest = (u16 *)dest;
		p_src = (u16 *)src;

		*p_dest = (*p_dest & 0xFF00) | (*p_src & 0x00FF);
	} else if ((dest % 2) && (src % 2)) {
		p_dest = (u16 *)(dest - 1);
		p_src = (u16 *)(src - 1);

		*p_dest = (*p_dest & 0x00FF) | (*p_src & 0xFF00);
	} else if (!(dest % 2) && (src % 2)) {
		p_dest = (u16 *)dest;
		p_src = (u16 *)(src - 1);

		*p_dest = (*p_dest & 0xFF00) | ((*p_src >> 8) & 0x00FF);
	} else if ((dest % 2) && !(src % 2)) {
		p_dest = (u16 *)(dest - 1);
		p_src = (u16 *)src;

		*p_dest = (*p_dest & 0x00FF) | ((*p_src << 8) & 0xFF00);
	} else {
		DPRAM_LOG_ERR("%s: Invalid case. \n", __func__);
	}
}

static inline void _memcpy(void *p_dest, const void *p_src, int size)
{
	unsigned long dest = (unsigned long)p_dest;
	unsigned long src = (unsigned long)p_src;

	if (size <= 0)
		return;

	if (dest & 1) {
		byte_align(dest, src);
		dest++, src++;
		size--;
	}

	if (size & 1) {
		byte_align(dest + size - 1, src + size - 1);
		size--;
	}

	if (src & 1) {
		unsigned char *s = (unsigned char *)src;
		volatile u16 *d = (unsigned short *)dest;

		size >>= 1;

		while (size--) {
			*d++ = s[0] | (s[1] << 8);
			s += 2;
		}
	} else {
		u16 *s = (u16 *)src;
		volatile u16 *d = (unsigned short *)dest;
		size >>= 1;
		while (size--)
			*d++ = *s++;
	}
}

/* Note the use of non-standard return values (0=match, 1=no-match) */
static inline int _memcmp(u8 *dest, u8 *src, int size)
{
#if 1
	while (size--)
		if (*dest++ != *src++)
			return 1;
	return 0;
#else
	int i = 0;
	for (i = 0 ; i < size ; i++)
		if (*(dest + i) != *(src + i))
			return 1;
	return 0;
#endif
}

void dpram_platform_init(void)
{

	/* LTE-STEALTH Related config. CS related settings are done in Machine Init*/
	unsigned int regVal;

	/* SIGNALS
	1) C210_DPRAM_nCS --> XM0CSN_0  (ie Xm0CSn[0] GPY_0[0])
	2) C210_OE_N -->XM0OEN
	3) C210_LB -> XM0BEN_0
	4) C210_UB --> XM0BEN_1
	5) C210_DPRAM_INT_N --> XEINT_8
	6) C210_WE_N --> XM0WEN
	7) DATA LINES --> XM0DATA_0 to XM0DATA_15
	8) Address Lines -->XM0ADDR_0 to XM0ADDR_13 
	*/
	regVal = __raw_readl (S5PV310_VA_CMU + 0xC940);    
	regVal |= 0x00000800; //CLK_SROMC [11]     
	writel(regVal, S5PV310_VA_CMU + 0xC940);

	regVal = __raw_readl ( S5P_SROM_BW);	
	regVal |= (0x00000008 | 0x00000004 | 0x00000001);	
	writel(regVal, S5P_SROM_BW);

	regVal = __raw_readl (S5P_SROM_BC0);	
	regVal = 0x22032220;	//0x04040000;	
	writel(regVal, S5P_SROM_BC0);	

	//1) C210_DPRAM_nCS --> XM0CSN_0  (ie Xm0CSn[0] GPY_0[0])
	//2) C210_OE_N -->XM0OEN
	//6) C210_WE_N --> XM0WEN
	regVal = __raw_readl(S5PV310_VA_GPIO2 + 0x0120);
	regVal = (regVal & 0xFF00FFF0) |0x00220002;
	writel(regVal, S5PV310_VA_GPIO2 + 0x0120);

	//3) C210_LB -> XM0BEN_0
	//4) C210_UB --> XM0BEN_1
	regVal = __raw_readl(S5PV310_VA_GPIO2 + 0x0140);
	regVal = (regVal & 0xFFFFF000) |0x00000222;
 	writel(regVal, S5PV310_VA_GPIO2 + 0x0140);

	
	//8) Address Lines -->XM0ADDR_0 to XM0ADDR_13 
	//ADDR LINES GPY3 and GPY4 //0x11000180  and 0x110001A0
	regVal = 0x22222222;
	writel(regVal, S5PV310_VA_GPIO2 + 0x0180);

	regVal = __raw_readl(S5PV310_VA_GPIO2 + 0x01A0);
	regVal = (regVal & 0xFF000000) |0x00222222;
	writel(regVal, S5PV310_VA_GPIO2 + 0x01A0);

	//7) DATA LINES --> XM0DATA_0 to XM0DATA_15
	//DATA LINES GPY5 and GPY6 //0x110001C0 and 0x110001E0
	regVal = 0x22222222;
	writel(regVal, S5PV310_VA_GPIO2 + 0x01C0);

	regVal = 0x22222222;
	writel(regVal, S5PV310_VA_GPIO2 + 0x01E0);

}

static inline int WRITE_TO_DPRAM_VERIFY(u32 dest, void *src, int size)
{
	int cnt = 3;

	while (cnt--) {
		_memcpy((void *)(DPRAM_VBASE + dest), (void *)src, size);
		if (!_memcmp((u8 *)(DPRAM_VBASE + dest), (u8 *)src, size))
			return 0;
	}
	DPRAM_LOG_ERR("%s: Verfication failed. \n", __func__);
	verification_condition_write++;
	return -1;
}

static inline int READ_FROM_DPRAM_VERIFY(void *dest, u32 src, int size)
{
	int cnt = 3;
	while (cnt--) {
		_memcpy((void *)dest, (void *)(DPRAM_VBASE + src), size);
		if (!_memcmp((u8 *)dest, (u8 *)(DPRAM_VBASE + src), size))
			return 0;
	}
	DPRAM_LOG_ERR("%s: Verfication failed. \n", __func__);
	verification_condition_read++;
	return -1;
}

static int dpram_get_lock_write(void)
{
	return atomic_read(&dpram_write_lock);
}

static int dpram_lock_write(void)
{    
	int lock_value;

	lock_value = atomic_inc_return(&dpram_write_lock);
	if ( lock_value != 1 )
	{
		DPRAM_LOG_INFO("lock_value (%d) != 1\n", lock_value);
		return -1;
	}

	return lock_value;
}

static void dpram_unlock_write(void)
{
	int lock_value;

	lock_value = atomic_dec_return(&dpram_write_lock);

	if ( lock_value != 0 )
		DPRAM_LOG_INFO("lock_value (%d) != 0\n", lock_value);
}


static void send_interrupt_to_phone(u16 irq_mask)
{
	{	/* Just for debugging against level-sensitive interrupts */
		u16 temp;
		READ_FROM_DPRAM(&temp, DPRAM_PDA2PHONE_INTERRUPT_ADDRESS,
				DPRAM_INTERRUPT_PORT_SIZE);
	}

	WRITE_TO_DPRAM(DPRAM_PDA2PHONE_INTERRUPT_ADDRESS,
			&irq_mask, DPRAM_INTERRUPT_PORT_SIZE);
#ifdef PRINT_SEND_IRQ
	DPRAM_LOG_SEND_IRQ("=====> send IRQ: %x \n", irq_mask);
#endif
}

/*We have to write complete pkts only to the dpram. Otherwise Via is getting memory corruption*/
/*Re-defining again here. Taken from multipdp.c. See if there is a more elegant way of doing this*/
struct pdp_hdr_dpram_nosof {
	u16	len;		/* Data length */// to Support  CDMA MODEM BINARY
	u8	id;			/* Channel ID */
	u8	control;	/* Control field */
} __attribute__ ((packed));

static int dpram_write(dpram_device_t *device, const unsigned char *buf, int len)
{
	int retval = 0;
	int size = 0;
//	int last_size = 0;
	u16 head, tail;
	u16 irq_mask = 0;
	u16 magic, access;
	struct pdp_hdr_dpram_nosof *pdp_hdr_ptr = (struct pdp_hdr_dpram_nosof *)(buf + 1);
//	int curr_pkt_len = 0, free_space = 0;
	u32 freesize = 0;
	u32 next_head = 0;

	/*  If the phone is down, let's reset everything and fail the write. */
	READ_FROM_DPRAM_VERIFY(&magic, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic));
	READ_FROM_DPRAM_VERIFY(&access, DPRAM_ACCESS_ENABLE_ADDRESS, sizeof(access));
	if (!dpram_phone_getstatus() || !access || magic != 0xAA) {
		/* We'll just call the dpram memory init again */
// for debugging start
		if(modem_condition_count<10){
			modem_condition_count++;	
			DPRAM_LOG_ERR("%s: id=%d, len=%d \n", __func__, pdp_hdr_ptr->id, pdp_hdr_ptr->len);
			DPRAM_LOG_ERR("%s: dpram_phone_getstatus=%d, access=%d, magic=%x \n", __func__, dpram_phone_getstatus(), access, magic);
		}
// for debugging end			
		dpram_initiate_self_error_correction();
//		return 0;
		return -1;
	}

	//Do sanity tests here on the buf. 
	// Note: Formatted data don't have channel ID. As we are taking length from packet, this should be okie
	if (buf[0] != 0x7F){
		DPRAM_LOG_ERR("%s: missing start of pkt 0x7F \n", __func__);
		return(-1);
	}
	if (pdp_hdr_ptr->len > (len - 2)){
		DPRAM_LOG_ERR("%s: Size mismatch. currPktLen=%d, len%d \n", __func__, pdp_hdr_ptr->len, len);
		return(-1);
	}
	if (buf[pdp_hdr_ptr->len + 1] != 0x7E){
		DPRAM_LOG_ERR("%s: missing end of pkt 0x7E \n", __func__);
		return(-1);
	}

	if ( dpram_lock_write() < 0 )
	{
		dpram_unlock_write();
		return -EAGAIN;
	}

	READ_FROM_DPRAM_VERIFY(&head, device->out_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->out_tail_addr, sizeof(tail));

	if(head < tail)
		freesize = tail - head - 1;
	else
		freesize = device->out_buff_size - head + tail -1;

	if(freesize >= len){

		next_head = head + len;

		if(next_head < device->out_buff_size) {
			size = len;
			WRITE_TO_DPRAM(device->out_buff_addr + head, buf, size);
			retval = size;

		}
		else {
			next_head -= device->out_buff_size;

			size = device->out_buff_size - head;
			WRITE_TO_DPRAM(device->out_buff_addr + head, buf, size);
			retval = size;

			size = next_head;
			WRITE_TO_DPRAM(device->out_buff_addr, buf + retval, size);
			retval += size;
		}

		/* update new head */
		head = next_head;
		WRITE_TO_DPRAM_VERIFY(device->out_head_addr, &head, sizeof(head));

	}

	dpram_unlock_write();

	if (retval <= 0) {
		MULTIPDP_LOG_ERR("%s: not enough space Fail (-1): freesize[%d],len[%d]\n",__func__, freesize, len);
		device->out_head_saved = head;
		device->out_tail_saved = tail;
		return -EAGAIN;
	}

	device->out_head_saved = head;
	device->out_tail_saved = tail;

	/* @LDK@ send interrupt to the phone, if.. */
	irq_mask = INT_MASK_VALID;

	if (retval > 0)
		irq_mask |= device->mask_send;

	if (len > retval)
		irq_mask |= device->mask_req_ack;

	send_interrupt_to_phone(irq_mask);	

	return len;
}

static inline int dpram_tty_insert_data(dpram_device_t *device, const u8 *psrc, u16 size)
{
#define CLUSTER_SEGMENT	1500
	u16 copied_size = 0;
	int retval = 0;
#ifdef PRINT_READ
	int i;
	DPRAM_LOG_READ("READ: %d\n", size);

	if(size<16)
	{
		for (i = 0; i < size; i++)
			DPRAM_LOG_READ( "IPC(FR):%02x ", *((unsigned char *)psrc + i));
	}
	else
	{
		DPRAM_LOG_READ(
		"IPC(FR):%02X %02X %02X %02x  %02X %02X %02X %02X  "
		"%02X %02X %02X %02X  %02X %02X %02X %02X\n",
		*(psrc+0), *(psrc+1), *(psrc+2), *(psrc+3), 
		*(psrc+4), *(psrc+5), *(psrc+6), *(psrc+7), 
		*(psrc+8), *(psrc+9), *(psrc+10), *(psrc+11), 
		*(psrc+12), *(psrc+13), *(psrc+14), *(psrc+15)
		);
	}
#endif
#ifdef PRINT_READ_SHORT
	DPRAM_LOG_READ_SHORT("READ: size:  %d\n", size);
#endif

	if (size > CLUSTER_SEGMENT && (device->serial.tty->index == 1)) {
		while (size) {
			copied_size = (size > CLUSTER_SEGMENT) ? CLUSTER_SEGMENT : size;
			tty_insert_flip_string(device->serial.tty, psrc + retval, copied_size);
			size -= copied_size;
			retval += copied_size;
		}
		return retval;
	}
	return tty_insert_flip_string(device->serial.tty, psrc, size);
}

static int dpram_read_fmt(dpram_device_t *device, const u16 non_cmd)
{

	int retval = 0;
	int retval_add = 0;
	int size = 0;
	u16 head, tail;

	READ_FROM_DPRAM_VERIFY(&head, device->in_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->in_tail_addr, sizeof(tail));

	DPRAM_LOG_READ("[DPRAM] Reading formatted queue...\n");
	DPRAM_LOG_READ("=====> %s,  head: %d, tail: %d\n", __func__, head, tail);

	if (head != tail) {
		u16 up_tail = 0;

		// ------- tail ++++++++++++ head -------- //
		if (head > tail) {
			size = head - tail;
			retval = dpram_tty_insert_data(device, (u8 *)(DPRAM_VBASE + (device->in_buff_addr + tail)), size);
			if (size != retval)
				DPRAM_LOG_ERR("(size != retval): size: %d, retval: %d \n", size, retval);
#ifdef PRINT_READ_SHORT
			else
				DPRAM_LOG_READ_SHORT("READ -return: %d\n", retval);
#endif
		} else {
			// +++++++ head ------------ tail ++++++++ //
			int tmp_size = 0;
			// Total Size.
			size = device->in_buff_size - tail + head;

			// 1. tail -> buffer end.
			tmp_size = device->in_buff_size - tail;
			retval = dpram_tty_insert_data(device, (u8 *)(DPRAM_VBASE + (device->in_buff_addr + tail)), tmp_size);
			if (tmp_size != retval)
				DPRAM_LOG_ERR("(tmp_size != retval): size: %d, retval: %d\n", size, retval);
#ifdef PRINT_READ_SHORT
			else
				DPRAM_LOG_READ_SHORT("READ -return: %d\n", retval);
#endif

			// 2. buffer start -> head.
			if (size > tmp_size) {
				retval_add = dpram_tty_insert_data(device, (u8 *)(DPRAM_VBASE + device->in_buff_addr), size - tmp_size);
				retval += retval_add;
				if ((size - tmp_size) != retval_add)
					DPRAM_LOG_ERR("((size - tmp_size) != retval_add): size - tmp_size: %d, retval_add: %d\n", size - tmp_size, retval_add);
#ifdef PRINT_READ_SHORT
				else
					DPRAM_LOG_READ_SHORT("READ -return_add: %d\n", retval_add);
#endif
			}
		}
		/* update tail */
		up_tail = (u16)((tail + retval) % device->in_buff_size);
		WRITE_TO_DPRAM_VERIFY(device->in_tail_addr, &up_tail, sizeof(up_tail));
	}
	else
	{
		DPRAM_LOG_ERR("%s, check!!! head: %d, tail: %d\n", __func__, head, tail);
	}
	
	device->in_head_saved = head;
	device->in_tail_saved = tail;

	if ( atomic_read(&fmt_txq_req_ack_rcvd) > 0 || (non_cmd & device->mask_req_ack) )
	{
		// there is a situation where the q become full after we reached the tasklet.
		// so this logic will allow us to send the RES_ACK as soon as we read 1 packet and CP get a chance to
		// write another buffer.
		DPRAM_LOG_RECV_IRQ("sending INT_MASK_RESP_ACK_F\n");
		send_interrupt_to_phone(INT_NON_COMMAND(device->mask_res_ack));
		atomic_set(&fmt_txq_req_ack_rcvd, 0);
		DPRAM_LOG_INFO("%s, fmt_txq_req_ack_rcvd: 0x%x\n", __func__, fmt_txq_req_ack_rcvd);
	}

	return retval;
}

static int dpram_read_raw(dpram_device_t *device, const u16 non_cmd)
{
	int retval = 0;
	int size = 0;
	u16 head, tail;
	u16 up_tail = 0;
	int ret;
	size_t len;
	struct pdp_info *dev = NULL;
	struct pdp_hdr hdr;
	u16 read_offset = 0;
	u8 len_high, len_low, id, control;
	u16 pre_data_size; //pre_hdr_size,
	u8 ch;

	READ_FROM_DPRAM_VERIFY(&head, device->in_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->in_tail_addr, sizeof(tail));

	if (head != tail) {

		up_tail = 0;

		if (head > tail)
			size = head - tail;				/* ----- (tail) 7f 00 00 7e (head) ----- */
		else
			size = device->in_buff_size - tail + head;	/* 00 7e (head) ----------- (tail) 7f 00 */

		read_offset = 0;

#ifdef PRINT_READ_SHORT
			DPRAM_LOG_READ_SHORT("RAW READ: head: %d, tail: %d, size: %d\n", head, tail, size);
#endif
		while (size)
		{            
			READ_FROM_DPRAM(&ch, device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size), sizeof(ch));

			if (ch == 0x7f) {
				read_offset++;
			} else {
				DPRAM_LOG_ERR("First byte: 0x%02x, drop bytes: %d, buff addr: 0x%08x, read addr: 0x%08x\n", 
					ch, size, (device->in_buff_addr), (device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size)));
				dpram_drop_data(device);
				return -1;
			}

			len_high = len_low = id = control = 0;
			READ_FROM_DPRAM(&len_low, device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size), sizeof(len_high));
			read_offset++;
			READ_FROM_DPRAM(&len_high, device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size), sizeof(len_low));
			read_offset++;
			READ_FROM_DPRAM(&id, device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size), sizeof(id));
			read_offset++;
			READ_FROM_DPRAM(&control, device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size), sizeof(control));
			read_offset++;

			hdr.len = len_high << 8 | len_low;
			hdr.id = id;
			hdr.control = control;

			len = hdr.len - sizeof(struct pdp_hdr);
			if (len <= 0) {
				DPRAM_LOG_ERR("READ RAW - wrong length, read_offset: %d, len: %d hdr.id: %d\n", read_offset, len, hdr.id);
				dpram_drop_data(device);
				return -1;


			}
			dev = pdp_get_dev(hdr.id);

#ifdef PRINT_READ_SHORT
			DPRAM_LOG_READ_SHORT("RAW READ: read_offset: %d, len: %d hdr.id: %d\n", read_offset, len, hdr.id);
#endif
			if (!dev)
			{
				DPRAM_LOG_ERR("onedram_raw_rx_data_callback : ch_id = %u, there is no existing device.\n", hdr.id);
				dpram_drop_data(device);
				return -1;
			}
			if (DEV_TYPE_SERIAL == dev->type)
			{
				if (dev->vs_dev.tty != NULL && dev->vs_dev.refcount)
				{

					if ((u16)(tail + read_offset) % device->in_buff_size + len < device->in_buff_size) {
						#ifdef LOCAL_LOOP_BACK_TEST
						if (dev->id == LOOP_BACK_CHANNEL) {
							// compare and resend , update stastic data
							//printk("receive loopback packet[%d]\n",loopback_res.nTransfered);
							if (loopback_ongoing) {
								if (strncmp((u8 *)(DPRAM_VBASE + (device->in_buff_addr + (u16)(tail + read_offset) % device->in_buff_size)), 
									loopback_data, loopback_res.nPacketDataSize))
									DPRAM_LOG_ERR("receive packet is not identical to that sent\n");
								else
									send_loop_back_packet(loopback_data, loopback_res.nPacketDataSize);
							}
							else
								DPRAM_LOG_ERR("loopback channel has gotten data, but test is no ongoing\n");
						}
						else {
						#endif		
						//dpram_debug_dump_raw_read_buffer((u8 *)(DPRAM_VBASE + (device->in_buff_addr + (u16)(tail + read_offset) % device->in_buff_size)), len);
						ret = tty_insert_flip_string(dev->vs_dev.tty, (u8 *)(DPRAM_VBASE + (device->in_buff_addr + (u16)(tail + read_offset) % device->in_buff_size)), len);
						tty_flip_buffer_push(dev->vs_dev.tty);
						#ifdef LOCAL_LOOP_BACK_TEST
						}
						#endif		
					} else {
						pre_data_size = device->in_buff_size - (tail + read_offset);
						#ifdef LOCAL_LOOP_BACK_TEST
						if (dev->id == LOOP_BACK_CHANNEL) {
							// compare and resend , update stastic data
							//printk("receive loopback packet[%d]\n",loopback_res.nTransfered);
							if (loopback_ongoing) {
								if (strncmp((u8 *)(DPRAM_VBASE + (device->in_buff_addr + (u16)(tail + read_offset) % device->in_buff_size)), 
									loopback_data, loopback_res.nPacketDataSize))
									DPRAM_LOG_ERR("receive packet is not identical to that sent\n");
								else
									send_loop_back_packet(loopback_data, loopback_res.nPacketDataSize);
							}
							else
								DPRAM_LOG_ERR("loopback channel has gotten data, but test is no ongoing\n");
						}
						else {
						#endif		
						//dpram_debug_dump_raw_read_buffer((u8 *)(DPRAM_VBASE + (device->in_buff_addr + tail + read_offset)), pre_data_size);
						ret = tty_insert_flip_string(dev->vs_dev.tty, (u8 *)(DPRAM_VBASE + (device->in_buff_addr + tail + read_offset)), pre_data_size);
						//dpram_debug_dump_raw_read_buffer((u8 *)(DPRAM_VBASE + (device->in_buff_addr)), len - pre_data_size);
						ret += tty_insert_flip_string(dev->vs_dev.tty, (u8 *)(DPRAM_VBASE + (device->in_buff_addr)), len - pre_data_size);
						tty_flip_buffer_push(dev->vs_dev.tty);
						#ifdef LOCAL_LOOP_BACK_TEST
						}
						#endif		
						//LOGL(DL_DEBUG, "RAW pre_data_size: %d, len-pre_data_size: %d, ret: %d\n", pre_data_size, len- pre_data_size, ret);
					}

#ifdef LOOP_BACK_TEST
					if (dev->id == 31){
						test_loopback_read();
					}
#endif
						
				} else
				{
					DPRAM_LOG_WARN("tty channel(id:%d) is not opened.\n", dev->id);
// for debugging start
					DPRAM_LOG_WARN("[dev->vs_dev.tty]: 0x%x, [dev->vs_dev.refcount]: %d\n", dev->vs_dev.tty, dev->vs_dev.refcount);
					if ((u16)(tail + read_offset) % device->in_buff_size + len < device->in_buff_size) {
						dpram_debug_dump_raw_read_buffer((u8 *)(DPRAM_VBASE + (device->in_buff_addr + (u16)(tail + read_offset) % device->in_buff_size)), len);
					} else {
						pre_data_size = device->in_buff_size - (tail + read_offset);
						dpram_debug_dump_raw_read_buffer((u8 *)(DPRAM_VBASE + (device->in_buff_addr + tail + read_offset)), pre_data_size);
						dpram_debug_dump_raw_read_buffer((u8 *)(DPRAM_VBASE + (device->in_buff_addr)), len - pre_data_size);
					}
// for debugging end			
					ret = len;
				}

				if (!ret)
				{
					DPRAM_LOG_ERR("(tty_insert_flip_string) drop byte: %d\n buff addr: %x\n read addr: %x\n", size, (device->in_buff_addr), (device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size)));
					dpram_drop_data(device);
					return -1;
				}
			}
			else if (DEV_TYPE_NET == dev->type)
			{

				DPRAM_LOG_ERR("%s, check!!! dev->type: %d\n", __func__, dev->type);

				if ((u16)(tail + read_offset) % device->in_buff_size + len < device->in_buff_size)
				{
					dpram_debug_dump_raw_read_buffer((u8 *)(DPRAM_VBASE + (device->in_buff_addr + (u16)(tail + read_offset) % device->in_buff_size)), len);
					ret = vnet_recv_rx_data((u8 *)(DPRAM_VBASE + (device->in_buff_addr + (u16)(tail + read_offset) % device->in_buff_size)), len, dev);
				} else
				{
					/* data span in two area. copy to local buffer and push. */
					/* TODO - Opimization later - avoid local copy here */
					pre_data_size = device->in_buff_size - (tail + read_offset);
					memcpy(pdp_net_rx_buf, (u8 *)(DPRAM_VBASE + (device->in_buff_addr + tail + read_offset)), pre_data_size);
					memcpy(pdp_net_rx_buf + pre_data_size, (u8 *)(DPRAM_VBASE + (device->in_buff_addr)), len - pre_data_size);
					dpram_debug_dump_raw_read_buffer(pdp_net_rx_buf, len);
					ret = vnet_recv_rx_data(pdp_net_rx_buf, len, dev);
				}
				if (ret != len)
				{
					DPRAM_LOG_ERR("vnet_recv_rx_data dropping bytes \n");
					dpram_drop_data(device);
					return -1;
				}
			}

			read_offset += ret;
			//LOGL(DL_DEBUG,"read_offset: %d ret= %d\n", read_offset, ret);

			READ_FROM_DPRAM(&ch, (device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size)), sizeof(ch));
			if (ch == 0x7e) {
				read_offset++;
			} else {
				DPRAM_LOG_ERR("Last byte: 0x%02x, drop bytes: %d, buff addr: 0x%08x, read addr: 0x%08x\n",
					ch, size, (device->in_buff_addr), (device->in_buff_addr + ((u16)(tail + read_offset) % device->in_buff_size)) );
				dpram_drop_data(device);
				return -1;
			}

			size -= (ret + sizeof(struct pdp_hdr) + 2);
			retval += (ret + sizeof(struct pdp_hdr) + 2);

			if (size < 0) {
				DPRAM_LOG_ERR("something wrong....\n");
				break;
			}
		}

		up_tail = (u16)((tail + read_offset) % device->in_buff_size);
		WRITE_TO_DPRAM_VERIFY(device->in_tail_addr, &up_tail, sizeof(up_tail));
	}
	else
	{
		DPRAM_LOG_ERR("%s, check!!! head: %d, tail: %d\n", __func__, head, tail);
	}

    
	device->in_head_saved = head;
	device->in_tail_saved = tail;

	if ( atomic_read(&raw_txq_req_ack_rcvd) > 0 || (non_cmd & device->mask_req_ack) )
	{
		// there is a situation where the q become full after we reached the tasklet.
		// so this logic will allow us to send the RES_ACK as soon as we read 1 packet and CP get a chance to
		// write another buffer.
		DPRAM_LOG_RECV_IRQ("sending INT_MASK_RESP_ACK_R\n");
		send_interrupt_to_phone(INT_NON_COMMAND(device->mask_res_ack));
		atomic_set(&raw_txq_req_ack_rcvd, 0);
		DPRAM_LOG_INFO("%s, raw_txq_req_ack_rcvd: 0x%x\n", __func__, raw_txq_req_ack_rcvd);		
	}

	return retval;
}

#ifdef _ENABLE_ERROR_DEVICE
void request_phone_reset(void)
{
	char buf[DPRAM_ERR_MSG_LEN];
	unsigned long flags;

	memset((void *)buf, 0, sizeof(buf));
	buf[0] = '9';
	buf[1] = ' ';

	memcpy(buf+2, "$PHONE-RESET", sizeof("$PHONE-RESET"));
	DPRAM_LOG_ERR("[PHONE ERROR] ->> %s\n", buf);

	local_irq_save(flags);
	memcpy(dpram_err_buf, buf, DPRAM_ERR_MSG_LEN);
	is_dpram_err = TRUE;
	local_irq_restore(flags);

	wake_up_interruptible(&dpram_err_wait_q);
	kill_fasync(&dpram_err_async_q, SIGIO, POLL_IN);
}
#endif

static int dpram_shared_bank_remap(void)
{

	dpram_base = ioremap_nocache(DPRAM_START_ADDRESS_PHYS + DPRAM_SHARED_BANK, DPRAM_SHARED_BANK_SIZE);
	if (dpram_base == NULL) {
		DPRAM_LOG_ERR("failed ioremap\n");
		return -ENOENT;
	}

	DPRAM_LOG_INFO("[DPRAM] ioremap returned %p\n", dpram_base);
	return 0;
}

static void dpram_clear(void)
{
	long /*i = 0,*/ err = 0;
	u16  i = 0;
	unsigned long flags;

	DPRAM_LOG_INFO("[DPRAM] *** entering dpram_clear()\n");
	/* clear DPRAM except interrupt area */
	local_irq_save(flags);

	for (i = DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS;
			i < DPRAM_SIZE - (DPRAM_INTERRUPT_PORT_SIZE * 2);
			i += 2) {
		*((u16 *)(DPRAM_VBASE + i)) = i;
	}
	local_irq_restore(flags);

	for (i = DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS;
			i < DPRAM_SIZE - (DPRAM_INTERRUPT_PORT_SIZE * 2);
			i += 2) {
		if (*((u16 *)(DPRAM_VBASE + i)) != i) {
			DPRAM_LOG_ERR("[DPRAM] *** dpram_clear() verification failed at %8.8X\n", (((unsigned int)DPRAM_VBASE) + i));
			if (err++ > 128)
				break;
		}
	}

	local_irq_save(flags);
	for (i = DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS;
			i < DPRAM_SIZE - (DPRAM_INTERRUPT_PORT_SIZE * 2);
			i += 2) {
		*((u16 *)(DPRAM_VBASE + i)) = 0;
	}
	local_irq_restore(flags);

	DPRAM_LOG_INFO("[DPRAM] *** leaving dpram_clear()\n");
}

static int dpram_init_and_report(void)
{
	const u16 magic_code = 0x00aa;
	u16 ac_code = 0x0000;
	const u16 init_end = INT_COMMAND(INT_MASK_CMD_INIT_END|INT_MASK_CP_AIRPLANE_BOOT|INT_MASK_CP_AP_ANDROID);

	/* @LDK@ write DPRAM disable code */
	WRITE_TO_DPRAM(DPRAM_ACCESS_ENABLE_ADDRESS, &ac_code, sizeof(ac_code));
	/* @LDK@ dpram clear */
	dpram_clear();
	/* @LDK@ write magic code */
	WRITE_TO_DPRAM(DPRAM_MAGIC_CODE_ADDRESS, &magic_code, sizeof(magic_code));
	/* @LDK@ write DPRAM enable code */
	ac_code = 0x0001;
	WRITE_TO_DPRAM(DPRAM_ACCESS_ENABLE_ADDRESS, &ac_code, sizeof(ac_code));

	/* @LDK@ send init end code to phone */
	send_interrupt_to_phone(init_end);

	DPRAM_LOG_INFO(" Send 0x%x to MailboxBA  (Dpram init finish).\n", init_end);

	phone_sync = 1;
	return 0;
}

static inline int dpram_get_read_available(dpram_device_t *device)
{
	u16 head, tail;
	READ_FROM_DPRAM_VERIFY(&head, device->in_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->in_tail_addr, sizeof(tail));
	/* printk(KERN_ERR "H: %d, T: %d, H-T: %d\n",head, tail, head-tail); */

	if (head < device->in_buff_size) {
		return head - tail;
	} else {
		dpram_drop_data(device);
		return 0;
	}
}


static void dpram_drop_data(dpram_device_t *device)
{
	u16 head, tail;
	READ_FROM_DPRAM_VERIFY(&head, device->in_head_addr, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, device->in_tail_addr, sizeof(tail));
	if (head >= device->in_buff_size || tail >= device->in_buff_size) {
		head = tail = 0;
		WRITE_TO_DPRAM_VERIFY(device->in_head_addr, &head, sizeof(head));
	}
	WRITE_TO_DPRAM_VERIFY(device->in_tail_addr, &head, sizeof(head));
	DPRAM_LOG_ERR("[DPRAM] %s, head: %d, tail: %d\n", __func__, head, tail);
}

static int dpram_phone_getstatus(void)
{
	return gpio_get_value(GPIO_PHONE_ACTIVE);
}

static ssize_t show_debug(struct device *d, struct device_attribute *attr, char *buf)
{
	int inbuf, outbuf;
	char *p = buf;

	u16 magic, enable;
	u16 fmt_in_head, fmt_in_tail, fmt_out_head, fmt_out_tail;
	u16 raw_in_head, raw_in_tail, raw_out_head, raw_out_tail;

	READ_FROM_DPRAM((void *)&magic, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic));
	READ_FROM_DPRAM((void *)&enable, DPRAM_ACCESS_ENABLE_ADDRESS, sizeof(enable));
	READ_FROM_DPRAM((void *)&fmt_in_head, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, sizeof(fmt_in_head));
	READ_FROM_DPRAM((void *)&fmt_in_tail, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, sizeof(fmt_in_tail));
	READ_FROM_DPRAM((void *)&fmt_out_head, DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS, sizeof(fmt_out_head));
	READ_FROM_DPRAM((void *)&fmt_out_tail, DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS, sizeof(fmt_out_tail));
	READ_FROM_DPRAM((void *)&raw_in_head, DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS, sizeof(raw_in_head));
	READ_FROM_DPRAM((void *)&raw_in_tail, DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS, sizeof(raw_in_tail));
	READ_FROM_DPRAM((void *)&raw_out_head, DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS, sizeof(raw_out_head));
	READ_FROM_DPRAM((void *)&raw_out_tail, DPRAM_PDA2PHONE_RAW_TAIL_ADDRESS, sizeof(raw_out_tail));

	inbuf = CIRC_CNT(fmt_in_head, fmt_in_tail, dpram_table[0].in_buff_size);
	outbuf = CIRC_CNT(fmt_out_head, fmt_out_tail, dpram_table[0].out_buff_size);
	p += sprintf(p, "%d\tSize\t%lu(in)\t%lu(out)\n"
			"\tIn\t%8u\t%8u\t%8u\n\tOut\t%8u\t%8u\t%8u\n",
			0, dpram_table[0].in_buff_size, dpram_table[0].out_buff_size,
			fmt_in_head, fmt_in_tail, inbuf,
			fmt_out_head, fmt_out_tail, outbuf);

	inbuf = CIRC_CNT(raw_in_head, raw_in_tail, dpram_table[1].in_buff_size);
	outbuf = CIRC_CNT(raw_out_head, raw_out_tail, dpram_table[1].out_buff_size);
	p += sprintf(p, "%d\tSize\t%lu(in)\t%lu(out)\n"
			"\tIn\t%8u\t%8u\t%8u\n\tOut\t%8u\t%8u\t%8u\n",
			1, dpram_table[1].in_buff_size, dpram_table[1].out_buff_size,
			raw_in_head, raw_in_tail, inbuf,
			raw_out_head, raw_out_tail, outbuf);

	return p - buf;

}
static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, show_debug, NULL);

static int dpram_send_irq_wait_resp(void)
{

#if !defined(USE_INTERRUPTABLE_LOAD)
	u16 in_interrupt = 0;
	int count=0;
#endif
	u16 out_interrupt = 0;

// Clear PHONE2PDA Interrupt 
	ClearPendingInterruptFromModem();

// Send PDA2PHONE interrupt
	out_interrupt = 0xDB12;
	send_interrupt_to_phone(out_interrupt);

#if defined(USE_INTERRUPTABLE_LOAD)
// TBD
#else
//	Wait Irq from CP
	while(1) {
		if(!gpio_get_value(GPIO_DPRAM_INT_N))
		{
			READ_FROM_DPRAM_VERIFY(&in_interrupt, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(in_interrupt));
			if(in_interrupt == 0xDBAB)
			{
				break;
			}
			else
			{
				DPRAM_LOG_ERR(" [in_interrupt]: 0x%08x \n", in_interrupt);
				DPRAM_LOG_ERR("dpram_send_irq_wait_resp fail. (in_interrupt != 0xDBAB)\n");
				return -1;
			}
		}
		msleep(1); //msleep(10);
		count++;
		if(count > 200)
		{
			DPRAM_LOG_ERR("dpram_send_irq_wait_resp fail. (count > 200)\n");
			return -1;
		}
	}
#endif

	return 0;

}

static int dpram_receive_irq_4upload(void)
{

	u16 in_interrupt = 0;
	int count=0;

//	Wait Irq from CP
#if defined(ENABLE_INTERRUPTABLE_UPLOAD)
	while(1) {
		if(check_param.upload_copy_complete)
		{
//			DPRAM_LOG_INFO(" [check_param.upload_copy_complete]: %d \n", check_param.upload_copy_complete);
			check_param.upload_copy_complete=0;
			break;
		}
		msleep(1);
		count++;
		if(count > 200)
		{
			DPRAM_LOG_ERR("dpram_receive_irq_4upload fail. (count > 200)\n");
			return -1;
		}
	}
#else
	while(1) {
		if(!gpio_get_value(GPIO_DPRAM_INT_N))
		{
			READ_FROM_DPRAM_VERIFY(&in_interrupt, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(in_interrupt));
			if(in_interrupt == 0xDBAB)
			{
				break;
			}
			else
			{
				DPRAM_LOG_ERR(" [in_interrupt]: 0x%08x \n", in_interrupt);
				DPRAM_LOG_ERR("dpram_receive_irq_4upload fail. (in_interrupt != 0xDBAB)\n");
				return -1;
			}
		}
		msleep(1); //msleep(10);
		count++;
		if(count > 200)
		{
			DPRAM_LOG_ERR("dpram_receive_irq_4upload fail. (count > 200)\n");
			return -1;
		}
	}
#endif /* ENABLE_INTERRUPTABLE_UPLOAD */

	return 0;

}

static int dpram_send_irq_4upload(void)
{

	u16 out_interrupt = 0;

// Clear PHONE2PDA Interrupt 
		ClearPendingInterruptFromModem();

// Send PDA2PHONE interrupt
		out_interrupt = 0xDB12;
		send_interrupt_to_phone(out_interrupt);

	return 0;

}

static int dpram_data_upload(struct _param_nv *param)
{

	int retval = 0;
	u16 size=0;

// Wait Irq from CP
	retval = dpram_receive_irq_4upload();

	if (retval) {
		DPRAM_LOG_ERR("dpram_receive_irq_4upload failed.\n");
		return -1;
	}

// Read data from buffer
	//SIZE
	READ_FROM_DPRAM(&param->size, DPRAM_DLOAD_SIZE_ADDRESS, sizeof(size));
	//31KB
	READ_FROM_DPRAM(param->addr, DPRAM_DLOAD_BUFF_ADDRESS, param->size);
	//TAG
	READ_FROM_DPRAM(&param->tag, DPRAM_DLOAD_TAG_ADDRESS, sizeof(size));
	//COUNT
	READ_FROM_DPRAM(&param->count, DPRAM_DLOAD_COUNT_ADDRESS, sizeof(size));

// Send interrupt
	retval = dpram_send_irq_4upload();

	if (retval) {
		DPRAM_LOG_ERR("dpram_send_irq_4upload failed.\n");
		return -1;
	}

	return retval;

}

static int dpram_data_load(struct _param_nv *param)
{

	int retval = 0;
	u16 size=0;
	int count=0;

	u16 data_size;
	unsigned int  send_size = 0;
	unsigned int  rest_size = 0;
	unsigned char *ptr;
	int i;

	if(param->size <= 0x7C00 ) /* 31KB = 31744byte = 0x7C00 */
	{

// Write data to buffer
		//31KB
		WRITE_TO_DPRAM(DPRAM_DLOAD_BUFF_ADDRESS, param->addr, param->size);
		//SIZE
		WRITE_TO_DPRAM(DPRAM_DLOAD_SIZE_ADDRESS, &param->size, sizeof(size));
		//TAG
		WRITE_TO_DPRAM(DPRAM_DLOAD_TAG_ADDRESS, &param->tag, sizeof(size));
		//COUNT
		WRITE_TO_DPRAM(DPRAM_DLOAD_COUNT_ADDRESS, &param->count, sizeof(size));

// Send interrupt
// Wait Irq from CP
		retval = dpram_send_irq_wait_resp();

		if (retval) {
			DPRAM_LOG_ERR("dpram_data_load failed.\n");
			return -1;
		}
		
	}
	else//Multi copy
	{

		ptr = param->addr;
		data_size = 0x7C00;
		rest_size = param->size;
		DPRAM_LOG_WARN("whole size: 0x%x\n", param->size);

		for (i = 0; send_size < param->size; i++) {
			if (rest_size < 0x7C00)
				data_size = rest_size;

// Write data to buffer
			//31KB
			WRITE_TO_DPRAM(DPRAM_DLOAD_BUFF_ADDRESS, ptr, data_size);
			//SIZE
			WRITE_TO_DPRAM(DPRAM_DLOAD_SIZE_ADDRESS, &data_size, sizeof(size));
			//TAG
			WRITE_TO_DPRAM(DPRAM_DLOAD_TAG_ADDRESS, &param->tag, sizeof(size));
			//COUNT
			count=i+1;
			WRITE_TO_DPRAM(DPRAM_DLOAD_COUNT_ADDRESS, &count, sizeof(size));

// Send interrupt
// Wait Irq from CP
			retval = dpram_send_irq_wait_resp();

			if (retval) {
				DPRAM_LOG_ERR("dpram_data_load failed.\n");
				return -1;
			}

			send_size += data_size;
			rest_size -= data_size;
			ptr += data_size;

			if (!(i % 100))
				DPRAM_LOG_ERR("[%d] 0x%x size done, rest size: 0x%x\n", i,
						send_size, rest_size);
		}

	}

	return retval;

}

static int dpram_phone_upload_step1(void)
{

	int retval = 0;
	int count=0;
	u16 in_interrupt = 0, out_interrupt = 0;

	DPRAM_LOG_WARN(" +---------------------------------------------+\n");
	DPRAM_LOG_WARN(" |            UPLOAD PHONE SDRAM               |\n");
	DPRAM_LOG_WARN(" +---------------------------------------------+\n");

#if defined(ENABLE_INTERRUPTABLE_UPLOAD)
	while(1) {
		if(check_param.upload_copy_start)
		{
			DPRAM_LOG_INFO(" [check_param.upload_copy_start]: %d \n", check_param.upload_copy_start);
			check_param.upload_copy_start=0;
			break;
		}
		msleep(1);
		count++;
		if(count > 200)
		{
			DPRAM_LOG_ERR("dpram_phone_upload_step1 fail. (count > 200)\n");
			return -1;
		}
	}
#else
	DPRAM_LOG_INFO(" Check gpio_get_value(GPIO_DPRAM_INT_N). \n");
	while(1) {

		if(!gpio_get_value(GPIO_DPRAM_INT_N))
		{
			READ_FROM_DPRAM(&in_interrupt, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(in_interrupt));
			DPRAM_LOG_WARN(" [in_interrupt]: 0x%04x \n", in_interrupt);
			if(in_interrupt == 0x1234)
			{
				break;
			}
			else
			{
				DPRAM_LOG_ERR("dpram_phone_upload_step1 fail. (in_interrupt != 0x1234)\n");
				return -1;
			}
		}
		msleep(1); //msleep(10);
		count++;
		if(count > 200)
		{
			DPRAM_LOG_ERR("dpram_phone_upload_step1 fail. (count > 200)\n");
			READ_FROM_DPRAM(&in_interrupt, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(in_interrupt));
			if(in_interrupt == 0x1234)
			{
				DPRAM_LOG_WARN(" [in_interrupt]: 0x%04x \n", in_interrupt);
				break;
			}
			return -1;
		}
	}
#endif /* ENABLE_INTERRUPTABLE_UPLOAD */

// Send interrupt -> '0xDEAD'
	out_interrupt = 0xDEAD;
	send_interrupt_to_phone(out_interrupt);

	return retval;

}

static int dpram_phone_upload_step2(struct _param_nv *param)
{

	int retval = 0;

// Read phone sdram for upload
	retval = dpram_data_upload(param);

	if (!(param->count % 500))
		DPRAM_LOG_WARN(" [param->count]:%d \n", param->count);
	
	if (param->tag == 4)
	{
		DPRAM_LOG_WARN(" [param->tag]:%d \n", param->tag);
	}

	if( retval < 0 ){
		DPRAM_LOG_ERR(" dpram_data_upload() is failed.\n");
		return -1;
	}

	return retval;

}

static int dpram_phone_image_load_prepare(void)
{
	int retval = 0;
	int count=0;
#if !defined(USE_INTERRUPTABLE_LOAD)
	u16 in_interrupt = 0;
#endif


#if defined(USE_INTERRUPTABLE_LOAD)

//	Wait Irq from CP
	while(1) {
		if(check_param.copy_start)
		{
			DPRAM_LOG_INFO(" [check_param.copy_start]: %d \n", check_param.copy_start);
			check_param.copy_start=0;
			break;
		}
		msleep(10);
		count++;
		if(count > 200)
		{
			DPRAM_LOG_ERR("dpram_phone_image_load fail. (count > 200)\n");
			return -1;
		}
	}
#else
	DPRAM_LOG_INFO(" Check gpio_get_value(GPIO_DPRAM_INT_N). \n");
	while(1) {

		if(!gpio_get_value(GPIO_DPRAM_INT_N))
		{
			READ_FROM_DPRAM(&in_interrupt, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(in_interrupt));
			DPRAM_LOG_INFO(" [in_interrupt]: 0x%04x \n", in_interrupt);
			if(in_interrupt == 0x1234)
			{
				break;
			}
			else
			{
				DPRAM_LOG_ERR("dpram_phone_image_load fail. (in_interrupt != 0x1234)\n");
				return -1;
			}
		}
		msleep(1); //msleep(10);
		count++;
		if(count > 200)
		{
			DPRAM_LOG_ERR("dpram_phone_image_load fail. (count > 200)\n");
			return -1;
		}
	}
#endif


	return retval;

}

static int dpram_phone_image_load(struct _param_nv *param)
{

	int retval = 0;
	int count=0;
#if !defined(USE_INTERRUPTABLE_LOAD)
	u16 in_interrupt = 0;
#endif

	DPRAM_LOG_INFO(" +---------------------------------------------+\n");
	DPRAM_LOG_INFO(" |                  LOAD PHONE IMAGE           |\n");
	DPRAM_LOG_INFO(" +---------------------------------------------+\n");
	DPRAM_LOG_INFO("  - Address of PHONE IMAGE data(virt): 0x%08x.\n", param->addr);
	DPRAM_LOG_INFO("  - Size of PHONE IMAGE data: %d.\n", param->size);
	DPRAM_LOG_INFO("  - Count of PHONE IMAGE data: %d.\n", param->count);

#if defined(USE_INTERRUPTABLE_LOAD)

//	Wait Irq from CP
//	while(1) {
//		if(check_param.copy_start)
//		{
//			DPRAM_LOG_INFO(" [check_param.copy_start]: %d \n", check_param.copy_start);
//			check_param.copy_start=0;
//			break;
//		}
//		msleep(10);
//		count++;
//		if(count > 200)
//		{
//			DPRAM_LOG_ERR("dpram_phone_image_load fail. (count > 200)\n");
//			return -1;
//		}
//	}

	img = (unsigned char *)vmalloc(param->size);
	if (img == NULL)
	{
		DPRAM_LOG_ERR("vmalloc() fail. \n");
		return -1;
	}   
	memset(img,0,param->size);
	memcpy(img, param->addr, param->size);

	data_param = kzalloc(sizeof(struct _param_nv), GFP_KERNEL);
	if (data_param == NULL)
	{
		DPRAM_LOG_ERR("kzalloc() fail. \n");
		vfree(img);
		return -1;
	}   
	
	check_param.total_size = param->size;
	check_param.rest_size = param->size;
	check_param.send_size = 0;
	check_param.copy_complete = 0;

	data_param->addr = img;
	data_param->size = 0x7C00;
//	data_param->count = 1;
	data_param->count = param->count;	
//	DPRAM_LOG_INFO(" [data_param->count]: %d. \n", data_param->count);

	data_param->tag = 0x0001;

	if (check_param.rest_size < 0x7C00)
		data_param->size = check_param.rest_size;

	retval = dpram_data_load(data_param);

//	Reset count
//	count =0;

//	Wait Irq from CP
	while(1) {
		if(check_param.copy_complete)
		{
			DPRAM_LOG_INFO(" [check_param.copy_complete]: %d \n", check_param.copy_complete);
			check_param.copy_complete=0;

			vfree(img);
			kfree(data_param);
		
			break;
		}
		msleep(10);
		count++;
		if(count > 1000)
		{
			DPRAM_LOG_ERR("dpram_phone_image_load fail. (count > 1000)\n");
			vfree(img);
			kfree(data_param);
			return -1;
		}
	}

#else
//	DPRAM_LOG_INFO(" Check gpio_get_value(GPIO_DPRAM_INT_N). \n");
//	while(1) {
//
//		if(!gpio_get_value(GPIO_DPRAM_INT_N))
//		{
//			READ_FROM_DPRAM(&in_interrupt, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(in_interrupt));
//			DPRAM_LOG_INFO(" [in_interrupt]: 0x%04x \n", in_interrupt);
//			if(in_interrupt == 0x1234)
//			{
//				break;
//			}
//			else
//			{
//				DPRAM_LOG_ERR("dpram_phone_image_load fail. (in_interrupt != 0x1234)\n");
//				return -1;
//			}
//		}
//		msleep(1); //msleep(10);
//		count++;
//		if(count > 200)
//		{
//			DPRAM_LOG_ERR("dpram_phone_image_load fail. (count > 200)\n");
//			return -1;
//		}
//	}

// Write 0x0001 to tag for image download
	param->tag=0x0001;
	retval = dpram_data_load(param);
#endif

	return retval;

}

static int dpram_nvdata_load(struct _param_nv *param)
{

	int retval = 0;
#if defined(USE_INTERRUPTABLE_LOAD)
	int count=0;
#endif

	DPRAM_LOG_INFO("PHONE NV LOAD (E) \n");

	DPRAM_LOG_INFO(" +---------------------------------------------+\n");
	DPRAM_LOG_INFO(" |                  LOAD NVDATA                |\n");
	DPRAM_LOG_INFO(" +---------------------------------------------+\n");
	DPRAM_LOG_INFO("  - Address of NV data(virt): 0x%08x.\n", param->addr);
	DPRAM_LOG_INFO("  - Size of NV data: %d.\n", param->size);
	DPRAM_LOG_INFO("  - Count of NV data: %d.\n", param->count);

#if defined(USE_INTERRUPTABLE_LOAD)

	img = (unsigned char *)vmalloc(param->size);
	if (img == NULL)
	{
		DPRAM_LOG_ERR("vmalloc(param->size) fail. \n");
		return -1;
	}   
	memset(img,0,param->size);
	memcpy(img, param->addr, param->size);

	data_param = kzalloc(sizeof(struct _param_nv), GFP_KERNEL);
	if (data_param == NULL)
	{
		DPRAM_LOG_ERR("kzalloc() fail. \n");
		vfree(img);
		return -1;
	}   

	check_param.total_size = param->size;
	check_param.rest_size = param->size;
	check_param.send_size = 0;
	check_param.copy_complete = 0;

	data_param->addr = img;
	data_param->size = 0x7C00;
	data_param->count = 1;
	data_param->tag = 0x0002;
//	DPRAM_LOG_INFO(" [data_param->count]: %d. \n", data_param->count);

	if (check_param.rest_size < 0x7C00)
		data_param->size = check_param.rest_size;

	retval = dpram_data_load(data_param);

//	Wait Irq from CP
	while(1) {
		if(check_param.copy_complete)
		{
			DPRAM_LOG_INFO(" [check_param.copy_complete]: %d \n", check_param.copy_complete);
			check_param.copy_complete=0;

			vfree(img);
			kfree(data_param);

			break;
		}
		msleep(10);
		count++;
		if(count > 200)
		{
			DPRAM_LOG_ERR("dpram_nvdata_load fail. (count > 200)\n");
			vfree(img);
			kfree(data_param);
			return -1;
		}
	}

#else
// Write 0x0001 to tag for image download
	param->tag=0x0002;
	retval = dpram_data_load(param);
#endif

	DPRAM_LOG_INFO("PHONE NV LOAD (X) \n");

	return retval;
	
}

static int dpram_phone_boot_start(void)
{

#if !defined(USE_INTERRUPTABLE_LOAD)
	u16 in_interrupt = 0;
#endif
	u16 out_interrupt = 0;
	int count=0;

	DPRAM_LOG_INFO(" +---------------------------------------------+\n");
	DPRAM_LOG_INFO(" |                  PHONE BOOT START           |\n");
	DPRAM_LOG_INFO(" +---------------------------------------------+\n");

// Send interrupt -> '0x4567'
	out_interrupt = 0x4567;
	send_interrupt_to_phone(out_interrupt);

#if defined(USE_INTERRUPTABLE_LOAD)

//	Wait Irq from CP
	while(1) {
		if(check_param.boot_complete)
		{
			DPRAM_LOG_INFO(" [check_param.boot_complete]: %d \n", check_param.boot_complete);
			check_param.boot_complete=0;
			break;
		}
		msleep(10);
		count++;
		if(count > 200)
		{
			DPRAM_LOG_ERR("dpram_phone_boot_start fail. (count > 200)\n");
			return -1;
		}
	}

#else
	DPRAM_LOG_INFO(" Check gpio_get_value(GPIO_DPRAM_INT_N). \n");

//	Wait irq from CP -> '0xABCD'
	while(1) {

		if(!gpio_get_value(GPIO_DPRAM_INT_N))
		{
			READ_FROM_DPRAM_VERIFY(&in_interrupt, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(in_interrupt));
			DPRAM_LOG_INFO(" [in_interrupt]: 0x%04x \n", in_interrupt);
			if(in_interrupt == 0xABCD)
			{
				break;
			}
			else
			{
				DPRAM_LOG_ERR("dpram_phone_boot_start fail. (in_interrupt != 0xABCD)\n");
				return -1;
			}
		}
		msleep(1); //msleep(10);
		count++;
		if(count > 1000)
		{
			DPRAM_LOG_ERR("dpram_phone_boot_start fail. (count > 1000)\n");
			return -1;
		}
	}
#endif

	return 0;
}

#if defined(USE_WAIT_4COMPLETE)
static int dpram_phone_boot_start_post_process(void)
{

	int count=0;

	DPRAM_LOG_INFO("dpram_phone_boot_start_post_process (E).\n");

//	Wait Irq from CP
	while(1) {
		if(boot_start_complete)
		{
			DPRAM_LOG_INFO(" boot_start_complete: %d \n", boot_start_complete);
			boot_start_complete=0;
			break;
		}
		msleep(10);
		count++;
		if(count > 200)
		{
			DPRAM_LOG_ERR("dpram_phone_boot_start_post_process fail. (count > 200)\n");
			return -1;
		}
	}

	DPRAM_LOG_INFO("dpram_phone_boot_start_post_process (X).\n");

	return 0;
}
#endif /* USE_WAIT_4COMPLETE */

#if defined(USE_INTERRUPTABLE_LOAD)
typedef struct interruptable_load_device {

	unsigned long interruptable_load_01;
	unsigned long interruptable_load_02;
	unsigned long interruptable_load_03;

} interruptable_load_device_t;

typedef struct interruptable_load_tasklet_data {
	interruptable_load_device_t *device;
	u_int16_t non_cmd;
} interruptable_load_tasklet_data_t;

static void interruptable_load_tasklet_handler(unsigned long data);

static DECLARE_TASKLET(interruptable_load_tasklet, interruptable_load_tasklet_handler, 0);

static void interruptable_load_tasklet_handler(unsigned long data)
{

	if(data_param == NULL)
	{
		DPRAM_LOG_ERR(" [%s]: Null pointer is detected. \n", __func__);
		return;
	}

	check_param.send_size += data_param->size;
	check_param.rest_size -= data_param->size;
	data_param->addr += data_param->size;

	if (check_param.send_size < check_param.total_size)
	{

		if (check_param.rest_size < 0x7C00)
		{
			data_param->size = check_param.rest_size;
			DPRAM_LOG_INFO(" [data_param->size]: 0x%08x. \n", data_param->size);
		}

		data_param->count += 1;
//		DPRAM_LOG_INFO(" [data_param->count]: %d. \n", data_param->count);

		dpram_data_load(data_param);
	}
	else
	{
		data_param->tag=0;
		check_param.copy_complete=1;
	}

}
#endif

#ifdef CONFIG_PROC_FS
static int dpram_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	char *p = page;
	int len;
	u16 magic, enable;
	u16 fmt_in_head, fmt_in_tail, fmt_out_head, fmt_out_tail;
	u16 raw_in_head, raw_in_tail, raw_out_head, raw_out_tail;
	u16 in_interrupt = 0, out_interrupt = 0;
	int fih, fit, foh, fot;
	int rih, rit, roh, rot;

#ifdef _ENABLE_ERROR_DEVICE
	char buf[DPRAM_ERR_MSG_LEN];
	unsigned long flags;
#endif	/* _ENABLE_ERROR_DEVICE */

	READ_FROM_DPRAM((void *)&magic, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic));
	READ_FROM_DPRAM((void *)&enable, DPRAM_ACCESS_ENABLE_ADDRESS, sizeof(enable));
	READ_FROM_DPRAM((void *)&fmt_in_head, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, sizeof(fmt_in_head));
	READ_FROM_DPRAM((void *)&fmt_in_tail, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, sizeof(fmt_in_tail));
	READ_FROM_DPRAM((void *)&fmt_out_head, DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS, sizeof(fmt_out_head));
	READ_FROM_DPRAM((void *)&fmt_out_tail, DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS, sizeof(fmt_out_tail));
	READ_FROM_DPRAM((void *)&raw_in_head, DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS, sizeof(raw_in_head));
	READ_FROM_DPRAM((void *)&raw_in_tail, DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS, sizeof(raw_in_tail));
	READ_FROM_DPRAM((void *)&raw_out_head, DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS, sizeof(raw_out_head));
	READ_FROM_DPRAM((void *)&raw_out_tail, DPRAM_PDA2PHONE_RAW_TAIL_ADDRESS, sizeof(raw_out_tail));


	fih = dpram_table[FORMATTED_INDEX].in_head_saved;
	fit = dpram_table[FORMATTED_INDEX].in_tail_saved;
	foh = dpram_table[FORMATTED_INDEX].out_head_saved;
	fot = dpram_table[FORMATTED_INDEX].out_tail_saved;
	rih = dpram_table[RAW_INDEX].in_head_saved;
	rit = dpram_table[RAW_INDEX].in_tail_saved;
	roh = dpram_table[RAW_INDEX].out_head_saved;
	rot = dpram_table[RAW_INDEX].out_tail_saved;

	READ_FROM_DPRAM((void *)&out_interrupt, DPRAM_PDA2PHONE_INTERRUPT_ADDRESS, sizeof(out_interrupt));
	READ_FROM_DPRAM((void *)&in_interrupt, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(in_interrupt));

#ifdef _ENABLE_ERROR_DEVICE
	memset((void *)buf, '\0', DPRAM_ERR_MSG_LEN);
	local_irq_save(flags);
	memcpy(buf, dpram_err_buf, DPRAM_ERR_MSG_LEN - 1);
	local_irq_restore(flags);
#endif	/* _ENABLE_ERROR_DEVICE */
	p += sprintf(p,
			"-------------------------------------\n"
			"| NAME\t\t\t| VALUE\n"
			"-------------------------------------\n"
			"|R MAGIC CODE\t\t| 0x%04x\n"
			"|R ENABLE CODE\t\t| 0x%04x\n"
			"|R PHONE->PDA FMT HEAD\t| %u\n"
			"|R PHONE->PDA FMT TAIL\t| %u\n"
			"|R PDA->PHONE FMT HEAD\t| %u\n"
			"|R PDA->PHONE FMT TAIL\t| %u\n"
			"|R PHONE->PDA RAW HEAD\t| %u\n"
			"|R RPHONE->PDA RAW TAIL\t| %u\n"
			"|R PDA->PHONE RAW HEAD\t| %u\n"
			"|R PDA->PHONE RAW TAIL\t| %u\n"

			"| FMT PHONE->PDA HEAD\t| %d\n"
			"| FMT PHONE->PDA TAIL\t| %d\n"
			"| FMT PDA->PHONE HEAD\t| %d\n"
			"| FMT PDA->PHONE TAIL\t| %d\n"
			"-------------------------------------\n"

			"| RAW PHONE->PDA HEAD\t| %d\n"
			"| RAW PHONE->PDA TAIL\t| %d\n"
			"| RAW PDA->PHONE HEAD\t| %d\n"
			"| RAW PDA->PHONE TAIL\t| %d\n"

			"-------------------------------------\n"

			"| PHONE->PDA IRQREG\t| 0x%04x\n"
			"| PDA->PHONE IRQREG\t| 0x%04x\n"

			"-------------------------------------\n"

#ifdef _ENABLE_ERROR_DEVICE
			"| LAST PHONE ERR MSG\t| %s\n"
#endif	/* _ENABLE_ERROR_DEVICE */

			"| PHONE ACTIVE\t\t| %s\n"
			"| DPRAM INT Level\t| %d\n"
			"-------------------------------------\n",
		magic, enable,
		fmt_in_head, fmt_in_tail, fmt_out_head, fmt_out_tail,
		raw_in_head, raw_in_tail, raw_out_head, raw_out_tail,
		fih, fit, foh, fot,
		rih, rit, roh, rot,
		in_interrupt, out_interrupt,
#ifdef _ENABLE_ERROR_DEVICE
		(buf[0] != '\0' ? buf : "NONE"),
#endif	/* _ENABLE_ERROR_DEVICE */
		(dpram_phone_getstatus() ? "ACTIVE" : "INACTIVE"),
		gpio_get_value(GPIO_DPRAM_INT_N));
	len = (p - page) - off;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + off;

	return len;
}
#endif /* CONFIG_PROC_FS */

/* dpram tty file operations. */
static int dpram_tty_open(struct tty_struct *tty, struct file *file)
{
	dpram_device_t *device = &dpram_table[tty->index];
	device->serial.tty = tty;
	device->serial.open_count++;
	DPRAM_LOG_WARN("[%s] %s opend\n", __func__, tty->name);
	DPRAM_LOG_WARN("[device->serial.tty]: 0x%x, [device->serial.open_count]: %d \n", device->serial.tty, device->serial.open_count);

	if (device->serial.open_count > 1) {
		device->serial.open_count--;
		DPRAM_LOG_ERR("[%s]: return -EBUSY \n",__FUNCTION__);
		return -EBUSY;
	}

	tty->driver_data = (void *)device;
	tty->low_latency = 1;
	return 0;
}

static void dpram_tty_close(struct tty_struct *tty, struct file *file)
{
	dpram_device_t *device = (dpram_device_t *)tty->driver_data;
	if (device && (device == &dpram_table[tty->index])) {
		down(&device->serial.sem);
		device->serial.open_count--;
		device->serial.tty = NULL;
		up(&device->serial.sem);
		DPRAM_LOG_WARN("[%s] %s closed\n", __func__, tty->name);
		DPRAM_LOG_WARN("[device->serial.tty]: 0x%x, [device->serial.open_count]: %d \n", device->serial.tty, device->serial.open_count);
	}
}

static int dpram_tty_write(struct tty_struct *tty,
		const unsigned char *buffer, int count)
{
	dpram_device_t *device = (dpram_device_t *)tty->driver_data;
	if (!device)
	{
		DPRAM_LOG_ERR("[%s] device not exist. [device]: 0x%x \n", __func__, device);
		return 0;
	}

	return dpram_write(device, buffer, count);
}

static int dpram_tty_write_room(struct tty_struct *tty)
{
	int avail;
	u16 head, tail;

	dpram_device_t *device = (dpram_device_t *)tty->driver_data;
	if (device != NULL) {
		head = device->out_head_saved;
		tail = device->out_tail_saved;
		avail = (head < tail) ? tail - head - 1 :
			device->out_buff_size + tail - head - 1;

		DPRAM_LOG_WRITE_SHORT("[DPRAM] %s: returns avail=%d\n", __func__, avail);

		return avail;
	}
	return 0;
}



static int dpram_tty_ioctl(struct tty_struct *tty, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	unsigned int val;

	//printk(KERN_ERR "[%s] 0x%x\n", __func__, cmd);

	switch (cmd) {

		case DPRAM_PHONE_GETSTATUS:
			val = dpram_phone_getstatus();
			return copy_to_user((unsigned int *)arg, &val, sizeof(val));

		case DPRAM_EXTRA_MEM_RW:
			/* do nothing here. Just a dummy call in Via6410 */
			return 0;

		case DPRAM_PHONE_UPLOAD_STEP1: 
			{

				printk(KERN_ERR "[%s] 0x%x\n", __func__, cmd);

#if !defined(ENABLE_INTERRUPTABLE_UPLOAD)
				/* @LDK@ unregister irq handler */
				free_irq(Gdpram_irq, NULL);
#endif /* ENABLE_INTERRUPTABLE_UPLOAD */

				val = dpram_phone_upload_step1();
				if( val < 0 ){
					DPRAM_LOG_ERR(" dpram_phone_upload_step1() is failed.\n");
					return -1;
				}

				return 0;
			}

		case DPRAM_PHONE_UPLOAD_STEP2: 
			{

				struct _param_nv param;

				val = copy_from_user((void *)&param, (void *)arg, sizeof(param));
				if( val < 0 ){
					DPRAM_LOG_ERR("[%s]  [STEP2] copy_from_user() is failed.\n", __func__);
					return -1;
				}

				val = dpram_phone_upload_step2(&param);
				if( val < 0 ){
					DPRAM_LOG_ERR(" dpram_phone_upload_step2() is failed.\n");
					return -1;
				}

				return copy_to_user((unsigned long *)arg, &param, sizeof(param));

			}

		case DPRAM_PHONE_POWON:
			{
				printk(KERN_ERR "[%s] 0x%x\n", __func__, cmd);

				return dpram_phone_image_load_prepare();
			}

		case DPRAM_PHONEIMG_LOAD:
			{
				struct _param_nv param;

				printk(KERN_ERR "[%s] 0x%x\n", __func__, cmd);

				val = copy_from_user((void *)&param, (void *)arg, sizeof(param));
				return dpram_phone_image_load(&param);
			}

		case DPRAM_NVDATA_LOAD:
			{
				struct _param_nv param;

				printk(KERN_ERR "[%s] 0x%x\n", __func__, cmd);

				val = copy_from_user((void *)&param, (void *)arg, sizeof(param));
				return dpram_nvdata_load(&param);
			}

		case DPRAM_PHONE_BOOTSTART:
			{
				int retval = 0;

				printk(KERN_ERR "[%s] 0x%x\n", __func__, cmd);

				retval=dpram_phone_boot_start();
				if (retval < 0) {
					DPRAM_LOG_ERR(" case DPRAM_PHONE_BOOTSTART is failed. \n");
					return -EFAULT;
				}

				// Reenable IRQ
#if !defined(USE_INTERRUPTABLE_LOAD)
				DPRAM_LOG_INFO(" Register register_interrupt_handler() \n");
				retval = register_interrupt_handler();
				if (retval < 0) {
					DPRAM_LOG_ERR(" register_interrupt_handler() is failed.\n");
					return -1;
				}
#endif /* USE_INTERRUPTABLE_LOAD */

#if defined(USE_WAIT_4COMPLETE)
				retval = dpram_phone_boot_start_post_process();				
				if (retval < 0) {
					DPRAM_LOG_ERR(" dpram_phone_boot_start_post_process() is failed.\n");
					return -1;
				}
#endif /* USE_WAIT_4COMPLETE */
				
				return 0;
			}

		default:
			printk(KERN_ERR "[%s] 0x%x\n", __func__, cmd);
			break;
	}
	return -ENOIOCTLCMD;
}

static int dpram_tty_chars_in_buffer(struct tty_struct *tty)
{
	int data;
	u16 head, tail;

	dpram_device_t *device = (dpram_device_t *)tty->driver_data;

	if (device != NULL) {
		head = device->out_head_saved;
		tail = device->out_tail_saved;
		data = (head > tail) ? head - tail - 1 :	device->out_buff_size - tail + head;

		DPRAM_LOG_WRITE_SHORT("[DPRAM] %s: returns data=%d\n", __func__, data);

		return data;
	}
	return 0;
}

#ifdef _ENABLE_ERROR_DEVICE
static int dpram_err_read(struct file *filp, char *buf, size_t count, loff_t *ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long flags;
	ssize_t ret;

	add_wait_queue(&dpram_err_wait_q, &wait);
	set_current_state(TASK_INTERRUPTIBLE);

	while (1) {
		local_irq_save(flags);

		if (is_dpram_err) {
			if (copy_to_user(buf, dpram_err_buf, count)) {
				ret = -EFAULT;
			} else {
				ret = count;
			}
			is_dpram_err = FALSE;
			local_irq_restore(flags);
			break;
		}
		local_irq_restore(flags);
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			break;
		}

		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}
		schedule();
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&dpram_err_wait_q, &wait);
	return ret;
}

static int dpram_err_fasync(int fd, struct file *filp, int mode)
{
	return fasync_helper(fd, filp, mode, &dpram_err_async_q);
}

static unsigned int dpram_err_poll(struct file *filp,
		struct poll_table_struct *wait)
{
	poll_wait(filp, &dpram_err_wait_q, wait);
	return ((is_dpram_err) ? (POLLIN | POLLRDNORM) : 0);
}
#endif	/* _ENABLE_ERROR_DEVICE */

/* handlers. */
static void res_ack_tasklet_handler(unsigned long data)
{
	dpram_device_t *device = (dpram_device_t *)data;

	/* TODO: check this case */
	u16 magic, access;
	READ_FROM_DPRAM_VERIFY(&magic, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic));
	READ_FROM_DPRAM_VERIFY(&access, DPRAM_ACCESS_ENABLE_ADDRESS, sizeof(access));
	if (!dpram_phone_getstatus() || !access || magic != 0xAA) {
// for debugging start
		if(modem_condition_count<10){
			modem_condition_count++;	
			DPRAM_LOG_ERR("%s: dpram_phone_getstatus=%d, access=%d, magic=%x \n", __func__, dpram_phone_getstatus(), access, magic);
		}
// for debugging end			
		dpram_initiate_self_error_correction();
		return;
	}

	if (device && device->serial.tty) {
		struct tty_struct *tty = device->serial.tty;

		if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) && tty->ldisc->ops->write_wakeup)
			(tty->ldisc->ops->write_wakeup)(tty);

		wake_up_interruptible(&tty->write_wait);
	}
// for debugging start
	else {
		DPRAM_LOG_ERR("[DPRAM] %s: Dropping data(1)...\n", __func__);
		DPRAM_LOG_ERR("[device]: 0x%x, [device->serial.tty]: 0x%x \n", device, device->serial.tty);
	}
// for debugging end			
	
}

static void fmt_rcv_tasklet_handler(unsigned long data)
{
	dpram_tasklet_data_t *tasklet_data = (dpram_tasklet_data_t *)data;

	dpram_device_t *device = tasklet_data->device;
	u16 non_cmd = tasklet_data->non_cmd;

	int ret = 0;
	int cnt = 0;

	/* TODO: check this case */
	u16 magic, access;
	READ_FROM_DPRAM_VERIFY(&magic, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic));
	READ_FROM_DPRAM_VERIFY(&access, DPRAM_ACCESS_ENABLE_ADDRESS, sizeof(access));
	if (!dpram_phone_getstatus() || !access || magic != 0xAA) {
// for debugging start
		if(modem_condition_count<10){
			modem_condition_count++;	
			DPRAM_LOG_ERR("%s: dpram_phone_getstatus=%d, access=%d, magic=%x \n", __func__, dpram_phone_getstatus(), access, magic);
		}
// for debugging end			
		dpram_initiate_self_error_correction();
		return;
	}

	if (device && device->serial.tty) {
		struct tty_struct *tty = device->serial.tty;

		while (dpram_get_read_available(device)) {
			ret = dpram_read_fmt(device, non_cmd);

			if (!ret)
				cnt++;

			if (cnt > 10) {
				dpram_drop_data(device);
				break;
			}
			if (ret < 0) {
				DPRAM_LOG_ERR("%s, dpram_read_fmt failed\n", __func__);
				/* TODO: ... wrong.. */
			}
			tty_flip_buffer_push(tty);
		}
	} else {
		DPRAM_LOG_ERR("[DPRAM] fmt_rcv_tasklet_handler(): Dropping data(2)...\n");
// for debugging start
		DPRAM_LOG_ERR("[device]: 0x%x, [device->serial.tty]: 0x%x \n", device, device->serial.tty);
// for debugging end			
		dpram_drop_data(device);
	}

	DPRAM_LOG_READ_SHORT("[DPRAM] Leaving fmt_rcv_tasklet_handler()\n");
}

static void raw_rcv_tasklet_handler(unsigned long data)
{
	dpram_tasklet_data_t *tasklet_data = (dpram_tasklet_data_t *)data;

	dpram_device_t *device = tasklet_data->device;
	u16 non_cmd = tasklet_data->non_cmd;

	int ret = 0;

	//printk("[HANG]enter raw_rcv \n");
	/* TODO: check this case */
	u16 magic, access;
	READ_FROM_DPRAM_VERIFY(&magic, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic));
	READ_FROM_DPRAM_VERIFY(&access, DPRAM_ACCESS_ENABLE_ADDRESS, sizeof(access));
	if (!dpram_phone_getstatus() || !access || magic != 0xAA) {
// for debugging start
		if(modem_condition_count<10){
			modem_condition_count++;	
			DPRAM_LOG_ERR("%s: dpram_phone_getstatus=%d, access=%d, magic=%x \n", __func__, dpram_phone_getstatus(), access, magic);
		}
// for debugging end			
		dpram_initiate_self_error_correction();
		return;
	}

	while (dpram_get_read_available(device)) {
		ret = dpram_read_raw(device, non_cmd);
		if (ret < 0) {
			DPRAM_LOG_ERR("RAW dpram_read_raw failed\n");
			/* TODO: ... wrong.. */
		}
	}

	//printk("[HANG]leave raw_rcv \n");
}

static void cmd_req_active_handler(void)
{
	send_interrupt_to_phone(INT_COMMAND(INT_MASK_CMD_RES_ACTIVE));
}

#ifdef _ENABLE_ERROR_DEVICE
static unsigned char cpdump_debug_file_name[DPRAM_ERR_MSG_LEN] = "CDMA Crash";
#endif	/* _ENABLE_ERROR_DEVICE */

static void cmd_error_display_handler(void)
{

#ifdef _ENABLE_ERROR_DEVICE
	char buf[DPRAM_ERR_MSG_LEN];
	unsigned long flags;

	memset((void *)buf, 0, sizeof(buf));

	// check the reset or crash
	if (!dpram_phone_getstatus()) {
		// --- can't catch the CDMA watchdog reset!!
		DPRAM_LOG_ERR("%s: Receive 0xC9 from CP. \n", __func__);

		DPRAM_LOG_ERR(" | PHONE ERR MSG\t| %s \n", cpdump_debug_file_name);

		READ_FROM_DPRAM(cpdump_debug_file_name
						, DPRAM_PHONE2PDA_FORMATTED_BUFFER_ADDRESS
						, sizeof(cpdump_debug_file_name));

		DPRAM_LOG_ERR(" | PHONE ERR MSG\t| %s \n", cpdump_debug_file_name);

	} else {
		buf[0] = 'C';
		buf[1] = 'D';
		buf[2] = 'M';
		buf[3] = 'A';
		buf[4] = ' ';

		READ_FROM_DPRAM((buf + 5), DPRAM_PHONE2PDA_FORMATTED_BUFFER_ADDRESS, sizeof (buf) - 6);
		READ_FROM_DPRAM(cpdump_debug_file_name
						, DPRAM_PHONE2PDA_FORMATTED_BUFFER_ADDRESS
						, sizeof(cpdump_debug_file_name));
	}

	local_irq_save(flags);
	local_irq_disable();
	
	memcpy(dpram_err_buf, buf, DPRAM_ERR_MSG_LEN);

	is_dpram_err = TRUE;

	local_irq_restore(flags);
#else
	char buf[151];

	memset((void *)buf, 0, sizeof(buf));

	// check the reset or crash
	if (!dpram_phone_getstatus()) {
		DPRAM_LOG_ERR("%s: Receive 0xC9 from CP. \n", __func__);
		DPRAM_LOG_ERR(" | PHONE ERR MSG\t| CDMA Crash \n");

		READ_FROM_DPRAM(buf, DPRAM_PHONE2PDA_FORMATTED_BUFFER_ADDRESS, (sizeof(buf)-1));

		DPRAM_LOG_ERR(" | PHONE ERR MSG\t| %s \n", buf);

		DPRAM_LOG_ERR("%s: modem_condition_count=%d \n", __func__, modem_condition_count);
		DPRAM_LOG_ERR("%s: verification_condition_write=%d \n", __func__, verification_condition_write);
		DPRAM_LOG_ERR("%s: verification_condition_read=%d \n", __func__, verification_condition_read);

	} else {
		buf[0] = 'C';
		buf[1] = 'D';
		buf[2] = 'M';
		buf[3] = 'A';
		buf[4] = ' ';

		READ_FROM_DPRAM((buf + 5), DPRAM_PHONE2PDA_FORMATTED_BUFFER_ADDRESS, sizeof (buf) - 6);

		DPRAM_LOG_ERR(" | PHONE ERR MSG\t| %s \n", buf);
	}

#endif	/* _ENABLE_ERROR_DEVICE */

}

static void cmd_phone_start_handler(void)
{
	DPRAM_LOG_INFO("[DPRAM] Received 0xc8 from Phone (Phone Boot OK).\n");
	dpram_init_and_report(); /* TODO init always now. otherwise starting/stopping modem code on debugger don't make this happen */
#if defined(USE_WAIT_4COMPLETE)
	boot_start_complete = 1;
#endif /* USE_WAIT_4COMPLETE */
}

static void command_handler(u16 cmd)
{
	DPRAM_LOG_WARN("[DPRAM] Entering command_handler(0x%04X)\n", cmd);
	switch (cmd) {
		case INT_MASK_CMD_REQ_ACTIVE:
			cmd_req_active_handler();
			break;

		case INT_MASK_CMD_ERR_DISPLAY:
			//TODO: add debug:
			cmd_error_display_handler();
#if defined(ENABLE_INTERRUPTABLE_UPLOAD)
			check_param.upload_copy_start= 0;
			check_param.upload_copy_complete= 0;
#endif /* ENABLE_INTERRUPTABLE_UPLOAD */
			break;

		case (INT_MASK_CMD_PHONE_START|INT_MASK_CP_QUALCOMM):
			cmd_phone_start_handler();
			break;

		default:
			DPRAM_LOG_ERR("Unknown command.. %x\n", cmd);
	}
//	DPRAM_LOG_INFO("[DPRAM] Leaving command_handler(0x%04X)\n", cmd);
}

static void non_command_handler(u16 non_cmd, void *dev_id)
{
	u16 head, tail;

	/* @LDK@ formatted check. */
	READ_FROM_DPRAM_VERIFY(&head, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, sizeof(tail));

	if (head != tail) {
		non_cmd |= INT_MASK_SEND_F;

		if (non_cmd & INT_MASK_REQ_ACK_F)
		{
			atomic_inc(&fmt_txq_req_ack_rcvd);
			DPRAM_LOG_INFO("%s, fmt_txq_req_ack_rcvd: 0x%x\n", __func__, fmt_txq_req_ack_rcvd);
		}
	} else {
		if (non_cmd & INT_MASK_REQ_ACK_F)
		{
			DPRAM_LOG_WARN("FMT DATA EMPTY & REQ_ACK_F (non_cmd:0x%x)\n", non_cmd);
			send_interrupt_to_phone(INT_COMMAND(INT_MASK_RES_ACK_F));
			atomic_set(&fmt_txq_req_ack_rcvd, 0);
			DPRAM_LOG_WARN("%s, fmt_txq_req_ack_rcvd: 0x%x\n", __func__, fmt_txq_req_ack_rcvd);
		}
	}

	/* @LDK@ raw check. */
	READ_FROM_DPRAM_VERIFY(&head, DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS, sizeof(head));
	READ_FROM_DPRAM_VERIFY(&tail, DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS, sizeof(tail));

	if (head != tail) {
		non_cmd |= INT_MASK_SEND_R;

		if (non_cmd & INT_MASK_REQ_ACK_R)
		{
			atomic_inc(&raw_txq_req_ack_rcvd);
			DPRAM_LOG_INFO("%s, raw_txq_req_ack_rcvd: 0x%x\n", __func__, raw_txq_req_ack_rcvd);
		}
	} else {
		if (non_cmd & INT_MASK_REQ_ACK_R)
		{
			DPRAM_LOG_WARN("RAW DATA EMPTY & REQ_ACK_R (non_cmd:0x%x)\n", non_cmd);
			send_interrupt_to_phone(INT_NON_COMMAND(INT_MASK_RES_ACK_R));
			atomic_set(&raw_txq_req_ack_rcvd, 0);
			DPRAM_LOG_WARN("%s, raw_txq_req_ack_rcvd: 0x%x\n", __func__, raw_txq_req_ack_rcvd);
		}
	}

	/* @LDK@ +++ scheduling.. +++ */
	if (non_cmd & INT_MASK_SEND_F) {
		wake_lock_timeout(&dpram_wake_lock, HZ/2);
		dpram_tasklet_data[FORMATTED_INDEX].device = &dpram_table[FORMATTED_INDEX];
		dpram_tasklet_data[FORMATTED_INDEX].non_cmd = non_cmd;
		fmt_send_tasklet.data = (unsigned long)&dpram_tasklet_data[FORMATTED_INDEX];
		tasklet_schedule(&fmt_send_tasklet);
	}

	if (non_cmd & INT_MASK_SEND_R) {
		wake_lock_timeout(&dpram_wake_lock, HZ*4);
		dpram_tasklet_data[RAW_INDEX].device = &dpram_table[RAW_INDEX];
		dpram_tasklet_data[RAW_INDEX].non_cmd = non_cmd;
		raw_send_tasklet.data = (unsigned long)&dpram_tasklet_data[RAW_INDEX];
		//printk("[HANG] sch raw tasklet \n");
		tasklet_hi_schedule(&raw_send_tasklet);
	}

	if (non_cmd & INT_MASK_RES_ACK_F) {
		wake_lock_timeout(&dpram_wake_lock, HZ/2);
		tasklet_schedule(&fmt_res_ack_tasklet);
	}

	if (non_cmd & INT_MASK_RES_ACK_R) {
		wake_lock_timeout(&dpram_wake_lock, HZ*4);
		tasklet_hi_schedule(&raw_res_ack_tasklet);
	}
}

/* @LDK@ interrupt handlers. */
static irqreturn_t dpram_irq_handler(int irq, void *dev_id)
{
	unsigned long flags;
	u16 irq_mask = 0;

	local_irq_save(flags);
	local_irq_disable();

	READ_FROM_DPRAM_VERIFY(&irq_mask, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(irq_mask));
	DPRAM_LOG_RECV_IRQ("received mailboxAB = 0x%x\n", irq_mask);

#if 0	// TODO:debug: print head tail
	u16 fih, fit, foh, fot;
	u16 rih, rit, roh, rot;
	u16 magic, access;

	READ_FROM_DPRAM_VERIFY(&magic, DPRAM_MAGIC_CODE_ADDRESS, sizeof(magic));
	READ_FROM_DPRAM_VERIFY(&access, DPRAM_ACCESS_ENABLE_ADDRESS, sizeof(access));

	READ_FROM_DPRAM_VERIFY(&fih, DPRAM_PHONE2PDA_FORMATTED_HEAD_ADDRESS, sizeof(fih));
	READ_FROM_DPRAM_VERIFY(&fit, DPRAM_PHONE2PDA_FORMATTED_TAIL_ADDRESS, sizeof(fit));
	READ_FROM_DPRAM_VERIFY(&foh, DPRAM_PDA2PHONE_FORMATTED_HEAD_ADDRESS, sizeof(foh));
	READ_FROM_DPRAM_VERIFY(&fot, DPRAM_PDA2PHONE_FORMATTED_TAIL_ADDRESS, sizeof(fot));
	READ_FROM_DPRAM_VERIFY(&rih, DPRAM_PHONE2PDA_RAW_HEAD_ADDRESS, sizeof(rih));
	READ_FROM_DPRAM_VERIFY(&rit, DPRAM_PHONE2PDA_RAW_TAIL_ADDRESS, sizeof(rit));
	READ_FROM_DPRAM_VERIFY(&roh, DPRAM_PDA2PHONE_RAW_HEAD_ADDRESS, sizeof(roh));
	READ_FROM_DPRAM_VERIFY(&rot, DPRAM_PDA2PHONE_RAW_TAIL_ADDRESS, sizeof(rot));

	DPRAM_LOG_HEAD_TAIL("magic	= 0x%x, access	= 0x%x\n", magic, access);
//	DPRAM_LOG_HEAD_TAIL("\n fmt_in  H:%4d, T:%4d, M:%4d\n fmt_out H:%4d, T:%4d, M:%4d\n raw_in  H:%4d, T:%4d, M:%4d\n raw out H:%4d, T:%4d, M:%4d\n", fih, fit, DPRAM_PHONE2PDA_FORMATTED_BUFFER_SIZE, foh, fot, DPRAM_PDA2PHONE_FORMATTED_BUFFER_SIZE, rih, rit, DPRAM_PHONE2PDA_RAW_BUFFER_SIZE, roh, rot, DPRAM_PDA2PHONE_RAW_BUFFER_SIZE);
	DPRAM_LOG_HEAD_TAIL("fmt_in  H:%4d, T:%4d, M:%4d\n", fih, fit, DPRAM_PHONE2PDA_FORMATTED_BUFFER_SIZE);	
	DPRAM_LOG_HEAD_TAIL("fmt_out H:%4d, T:%4d, M:%4d\n", foh, fot, DPRAM_PDA2PHONE_FORMATTED_BUFFER_SIZE);	
	DPRAM_LOG_HEAD_TAIL("raw_in  H:%4d, T:%4d, M:%4d\n", rih, rit, DPRAM_PHONE2PDA_RAW_BUFFER_SIZE);
	DPRAM_LOG_HEAD_TAIL("raw out H:%4d, T:%4d, M:%4d\n", roh, rot, DPRAM_PDA2PHONE_RAW_BUFFER_SIZE);	
#endif

	if(irq_mask == 0x1234)
	{
		DPRAM_LOG_INFO(" Recieve irq_mask: 0x%04x\n", irq_mask);
#if defined(USE_INTERRUPTABLE_LOAD)
		check_param.copy_start = 1;
#endif /* USE_INTERRUPTABLE_LOAD */
#if defined(ENABLE_INTERRUPTABLE_UPLOAD)
		check_param.upload_copy_start = 1;
#endif /* ENABLE_INTERRUPTABLE_UPLOAD */

		local_irq_restore(flags);
		return IRQ_HANDLED;
	}

	if (irq_mask == 0xDBAB) {
//		DPRAM_LOG_INFO(" Recieve irq_mask: 0x%04x\n", irq_mask);
#if defined(USE_INTERRUPTABLE_LOAD)
		tasklet_schedule(&interruptable_load_tasklet);
#endif /* USE_INTERRUPTABLE_LOAD */

#if defined(ENABLE_INTERRUPTABLE_UPLOAD)
		check_param.upload_copy_complete=1;
#endif /* ENABLE_INTERRUPTABLE_UPLOAD */

		local_irq_restore(flags);
		return IRQ_HANDLED;
	} 

	if(irq_mask == 0xABCD) 
	{
		DPRAM_LOG_WARN(" Recieve irq_mask: 0x%04x\n", irq_mask);
#if defined(USE_INTERRUPTABLE_LOAD)
		check_param.boot_complete = 1;
#endif /* USE_INTERRUPTABLE_LOAD */
		local_irq_restore(flags);
		return IRQ_HANDLED;
	} 

	// valid bit verification. @LDK@
	if (!(irq_mask & INT_MASK_VALID)) {
		DPRAM_LOG_ERR("Invalid interrupt mask: 0x%04x\n", irq_mask);
		DPRAM_LOG_ERR("[DPRAM] Leaving dpram_irq_handler()\n");
		local_irq_restore(flags);
		return IRQ_HANDLED;
	}

	// Say something about the phone being dead...
	if (irq_mask == INT_POWERSAFE_FAIL) {
		DPRAM_LOG_ERR("[DPRAM] *** MODEM image corrupt.  Rerun download. ***\n");
		local_irq_restore(flags);
		return IRQ_HANDLED;
	}

	if (irq_mask & INT_MASK_COMMAND) {
		irq_mask &= ~(INT_MASK_VALID | INT_MASK_COMMAND);
		wake_lock_timeout(&dpram_wake_lock, HZ/2);
		command_handler(irq_mask);
	} else {
		irq_mask &= ~INT_MASK_VALID;
		non_command_handler(irq_mask, dev_id);
	}

//	DPRAM_LOG_INFO("[DPRAM] Leaving dpram_irq_handler()\n");
	local_irq_restore(flags);
	return IRQ_HANDLED;
}

/* basic functions. */
#ifdef _ENABLE_ERROR_DEVICE
static const struct file_operations dpram_err_ops = {
	.owner = THIS_MODULE,
	.read = dpram_err_read,
	.fasync = dpram_err_fasync,
	.poll = dpram_err_poll,
	.llseek = no_llseek,
	/* TODO: add more operations */
};
#endif	/* _ENABLE_ERROR_DEVICE */

static struct tty_operations dpram_tty_ops = {
	.open 		= dpram_tty_open,
	.close 		= dpram_tty_close,
	.write 		= dpram_tty_write,
	.write_room = dpram_tty_write_room,
	.ioctl 		= dpram_tty_ioctl,
	.chars_in_buffer = dpram_tty_chars_in_buffer,
	/* TODO: add more operations */
};

#ifdef _ENABLE_ERROR_DEVICE
static void unregister_dpram_err_device(void)
{
	unregister_chrdev(DRIVER_MAJOR_NUM, DPRAM_ERR_DEVICE);
	class_destroy(dpram_class);
}

static int register_dpram_err_device(void)
{
	/* @LDK@ 1 = formatted, 2 = raw, so error device is '0' */
	struct device *dpram_err_dev_t;
	int ret = register_chrdev(DRIVER_MAJOR_NUM, DPRAM_ERR_DEVICE, &dpram_err_ops);
	if (ret < 0)
		return ret;

	dpram_class = class_create(THIS_MODULE, "dpramerr");
	if (IS_ERR(dpram_class)) {
		unregister_dpram_err_device();
		return -EFAULT;
	}

	dpram_err_dev_t = device_create(dpram_class, NULL,
			MKDEV(DRIVER_MAJOR_NUM, 0), NULL, DPRAM_ERR_DEVICE);

	if (IS_ERR(dpram_err_dev_t)) {
		unregister_dpram_err_device();
		return -EFAULT;
	}
	return 0;
}
#endif	/* _ENABLE_ERROR_DEVICE */

static int register_dpram_driver(void)
{
	int retval = 0;
	/* @LDK@ allocate tty driver */
	dpram_tty_driver = alloc_tty_driver(MAX_INDEX);
	if (!dpram_tty_driver)
		return -ENOMEM;

	/* @LDK@ initialize tty driver */
	dpram_tty_driver->owner = THIS_MODULE;
	dpram_tty_driver->magic = TTY_DRIVER_MAGIC;
	dpram_tty_driver->driver_name = DRIVER_NAME;
	dpram_tty_driver->name = "dpram";
	dpram_tty_driver->major = DRIVER_MAJOR_NUM;
	dpram_tty_driver->minor_start = 1;
	dpram_tty_driver->num = MAX_INDEX;
	dpram_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	dpram_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	dpram_tty_driver->flags = TTY_DRIVER_REAL_RAW;
	dpram_tty_driver->init_termios = tty_std_termios;
	dpram_tty_driver->init_termios.c_cflag =
		(B115200 | CS8 | CREAD | CLOCAL | HUPCL);

	tty_set_operations(dpram_tty_driver, &dpram_tty_ops);
	dpram_tty_driver->ttys = dpram_tty;
	dpram_tty_driver->termios = dpram_termios;
	dpram_tty_driver->termios_locked = dpram_termios_locked;

	/* @LDK@ register tty driver */
	retval = tty_register_driver(dpram_tty_driver);

	if (retval) {
		DPRAM_LOG_ERR("tty_register_driver error\n");
		put_tty_driver(dpram_tty_driver);
		return retval;
	}
	return 0;
}

static void unregister_dpram_driver(void)
{
	tty_unregister_driver(dpram_tty_driver);
}

/**** Virtual Network Interface functions */
static int  vnet_recv_rx_data(u8 *buf, int size, struct pdp_info *dev)
{
	int ret_val = 0;
	struct sk_buff *skb;
	unsigned char ip_hdr_byte1 = 0, ip_version = 0;

#ifdef SVNET_PDP_ETHER
	char *p;
	struct ethhdr *eth_hdr;
	char source[ETH_ALEN] = {18, 52, 86, 120, 154, 188};
	char dest[ETH_ALEN]= {18, 0,  0,  0,   0,   0};
#endif

	MULTIPDP_LOG_INFO("In vnet_recv_rx_data \n");

	if (!netif_running(dev->vn_dev.net))
	{
		MULTIPDP_LOG_ERR("%s(id: %u) is not running\n", dev->vn_dev.net->name, dev->id);
		ret_val = size; /* just say we cosumed the buffer */
		return ret_val;
	}

#ifdef SVNET_PDP_ETHER
	skb = alloc_skb(size+ sizeof(struct ethhdr), GFP_ATOMIC);
#else
	skb = alloc_skb(size, GFP_ATOMIC);
#endif
	if (skb == NULL)
	{
		MULTIPDP_LOG_ERR("vnet_recv_rx_data ==> alloc_skb() failed\n");
		return ret_val;
	}
	/* determine the ip version */
	ip_hdr_byte1 = (*buf);
	ip_version = ip_hdr_byte1 >> 4;

#ifdef SVNET_PDP_ETHER
	p = skb_put(skb, size + sizeof(struct ethhdr));
	eth_hdr = (struct ethhdr *)p;
	memcpy(eth_hdr->h_dest, dest, ETH_ALEN);
	memcpy(eth_hdr->h_source, source, ETH_ALEN);
	eth_hdr->h_proto = (ip_version == 0x06)? __constant_htons(ETH_P_IPV6): __constant_htons(ETH_P_IP);
   	p = p + sizeof(struct ethhdr);	/* advance eth header size */
	memcpy((void *)p, (void *)buf, size); /* copy payload from NIC perspective */
#else
	p = skb_put(skb, size);
	memcpy((void *)p, (void *)buf, size); /*no header, copy payload from begining */
#endif

	/* initalize skb net device information */
	skb->dev = dev->vn_dev.net;
	/* fill the protocol type in skb */
	skb->protocol = (ip_version == 0x06)? __constant_htons(ETH_P_IPV6): __constant_htons(ETH_P_IP);

#ifdef SVNET_PDP_ETHER
	skb_reset_mac_header(skb);
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	skb_pull(skb, sizeof(struct ethhdr));
#endif
	   
	netif_rx(skb); /* push the allocated skb to higher level */
	dev->vn_dev.stats.rx_packets++;
	dev->vn_dev.stats.rx_bytes += skb->len;

	return size;
}

static void vnet_del_dev(struct net_device *net)
{
	unregister_netdev(net);
	kfree(net);
}

static int vnet_open(struct net_device *net)
{
	struct pdp_info *dev = (struct pdp_info *)net->ml_priv;

	INIT_WORK(&dev->vn_dev.xmit_work_struct, NULL);

	down(&dev->vn_dev.netq_sem);
	netif_start_queue(net);
	if(dev->vn_dev.netq_active == SUSPEND)
		netif_stop_queue(net);
	dev->vn_dev.netq_init = 1;
	up(&dev->vn_dev.netq_sem);

	return 0;
}

static int vnet_stop(struct net_device *net)
{
	netif_stop_queue(net);
	flush_scheduled_work(); /* flush any pending tx tasks */

	return 0;
}

static void vnet_defer_xmit(struct work_struct *data)
{
	struct vnet_struct *vnet_ptr = NULL;
	struct sk_buff *skb=NULL;
	vnet_ptr = container_of(data, struct vnet_struct, xmit_work_struct);
	skb = vnet_ptr->skb_ptr;
	struct net_device *net = (struct net_device *)skb->dev;
	struct pdp_info *dev = (struct pdp_info *)net->ml_priv;
	int ret = 0;

	MULTIPDP_LOG_INFO("In vnet_defer_xmit\n");
	MULTIPDP_LOG_WRITE("WRITE vnet_defer_xmit ==> Len %d \n", skb->len);

	ret = pdp_mux(dev, skb->data, skb->len);

	if (ret < 0) {
		dev->vn_dev.stats.tx_dropped++;
	}

	else {
		net->trans_start = jiffies;
		dev->vn_dev.stats.tx_bytes += skb->len;
		dev->vn_dev.stats.tx_packets++;
	}

	dev_kfree_skb_any(skb);
	down(&dev->vn_dev.netq_sem);
	if (dev->vn_dev.netq_active == ACTIVE){
		netif_wake_queue(net);
	}
	up(&dev->vn_dev.netq_sem);
}

static int vnet_start_xmit(struct sk_buff *skb, struct net_device *net)
{
//	int i;
	struct pdp_info *dev = (struct pdp_info *)net->ml_priv;

	MULTIPDP_LOG_INFO("In vnet_start_xmit\n");

	MULTIPDP_LOG_WRITE("WRITE vnet_start_xmit ==> Buff Len %d \n", skb->len);

#ifdef SVNET_PDP_ETHER
	skb_pull(skb,14);
#endif

	dev->vn_dev.skb_ptr = skb;

	PREPARE_WORK(&dev->vn_dev.xmit_work_struct,vnet_defer_xmit);

	MULTIPDP_LOG_WRITE("WRITE vnet_start_xmit ==> schedule write work item \n");

	schedule_work(&dev->vn_dev.xmit_work_struct);
	netif_stop_queue(net);

	return 0;
}

static struct net_device_stats *vnet_get_stats(struct net_device *net)
{
	struct pdp_info *dev = (struct pdp_info *)net->ml_priv;
	return &dev->vn_dev.stats;
}

static void vnet_tx_timeout(struct net_device *net)
{
	struct pdp_info *dev = (struct pdp_info *)net->ml_priv;

	net->trans_start = jiffies;
	dev->vn_dev.stats.tx_errors++;

	down(&dev->vn_dev.netq_sem);
	if (dev->vn_dev.netq_active == ACTIVE){
		netif_wake_queue(net);
	}
	up(&dev->vn_dev.netq_sem);
	MULTIPDP_LOG_ERR("vnet_tx_timeout ==>Tx timed out\n");
}

static const struct net_device_ops pdp_netdev_ops = {
	.ndo_open		= vnet_open,
	.ndo_stop		= vnet_stop,
	.ndo_start_xmit	= vnet_start_xmit,
	.ndo_get_stats	= vnet_get_stats,
	.ndo_tx_timeout	= vnet_tx_timeout,
};

static void vnet_setup(struct net_device *dev)
{
	dev->netdev_ops = &pdp_netdev_ops;
	
#ifdef SVNET_PDP_ETHER
	dev->type = ARPHRD_ETHER;
#else
	dev->type = ARPHRD_PPP;
#endif

#ifdef SVNET_PDP_ETHER
	dev->addr_len = ETH_ALEN;
	dev->dev_addr[0] = 0x12;
#else
	dev->addr_len = 0;
#endif
	dev->hard_header_len 	= 0;
	dev->mtu		= MAX_PDP_DATA_LEN;
	dev->tx_queue_len	= 1000;
#ifdef SVNET_PDP_ETHER
	dev->flags		= IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST | IFF_SLAVE;
#else
	dev->flags		= IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST;
#endif
	dev->watchdog_timeo	= 5 * HZ;
}

static void vnet_setup_v6(struct net_device *dev)
{
	dev->netdev_ops = &pdp_netdev_ops;
	
#ifdef SVNET_PDP_ETHER
	dev->type = ARPHRD_ETHER;
#else
	dev->type = ARPHRD_PPP;
#endif
	dev->hard_header_len 	= 0;
	dev->mtu		= MAX_PDP_DATA_LEN;
#ifdef SVNET_PDP_ETHER
	dev->addr_len = ETH_ALEN;
	dev->dev_addr[0] = 0x12;
#else
	dev->addr_len = 0;
#endif
	dev->tx_queue_len	= 1000;
#ifdef SVNET_PDP_ETHER
	dev->flags		= IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST | IFF_SLAVE;
#else
	dev->flags		= IFF_POINTOPOINT | IFF_NOARP | IFF_MULTICAST;
#endif
	dev->watchdog_timeo	= 5 * HZ;
}

static struct net_device *vnet_add_dev(void *priv)
{
	int ret;
	struct net_device *dev;

	MULTIPDP_LOG_INFO(" %s\n",__FUNCTION__);
	dev = alloc_netdev(0, "hrpd%d", vnet_setup);
	if (dev == NULL) {
		MULTIPDP_LOG_ERR("vnet_add_dev ==> alloc_netdev failed\n");
		return NULL;
	}

	dev->ml_priv		= priv;
	ret = register_netdev(dev);

	if (ret != 0) {
		MULTIPDP_LOG_ERR("vnet_add_dev ==> register_netdevice failed: %d\n", ret);
		kfree(dev);
		return NULL;
	}
	return dev;
}

static struct net_device *vnet_add_dev_v6(void *priv)
{
	int ret;
	struct net_device *dev;

	MULTIPDP_LOG_INFO(" %s\n",__FUNCTION__);
	dev = alloc_netdev(0, "hrpd%d", vnet_setup_v6);
	if (dev == NULL) {
		MULTIPDP_LOG_ERR(" vnet_add_dev_v6 ==> out of memory\n");
		return NULL;
	}
	dev->ml_priv		= priv;

	ret = register_netdev(dev);

	if (ret != 0) {
		MULTIPDP_LOG_ERR("vnet_add_dev_v6 ==> register_netdevice failed: %d\n", ret);
		kfree(dev);
		return NULL;
	}
	return dev;
}

/******* Virtual Serial Interface functions *******/
static int vs_open(struct tty_struct *tty, struct file *filp)
{

	struct pdp_info *dev;

	dev = pdp_get_serdev(tty->driver->name); /* 2.6 kernel porting */
	if (dev == NULL)
	{
		MULTIPDP_LOG_ERR("[%s]: return -ENODEV \n",__FUNCTION__);
		return -ENODEV;
	}

	tty->driver_data = (void *)dev;
	tty->low_latency = 1;
	dev->vs_dev.tty = tty;
	dev->vs_dev.refcount++;
	MULTIPDP_LOG_WARN("[%s] %s, dev->vs_dev.tty: 0x%x, refcount: %d \n", __func__, tty->driver->name, dev->vs_dev.tty, dev->vs_dev.refcount);
	MULTIPDP_LOG_WARN("pid: (%s,%d) \n", current->comm,current->pid); 
	return 0;

}

static void vs_close(struct tty_struct *tty, struct file *filp)
{
	struct pdp_info *dev;

	dev = pdp_get_serdev(tty->driver->name);
	if (!dev)
	{
		MULTIPDP_LOG_ERR("[%s]: (!dev) \n",__FUNCTION__);
		return;
	}
	dev->vs_dev.refcount--;
	MULTIPDP_LOG_WARN("[%s] %s, refcount: %d \n", __func__, tty->driver->name, dev->vs_dev.refcount);
	MULTIPDP_LOG_WARN("pid: (%s,%d) \n", current->comm,current->pid); 
	return;
}

static int vs_write(struct tty_struct *tty,
		const unsigned char *buf, int count)
{
	int ret;
	struct pdp_info *dev = (struct pdp_info *)tty->driver_data;

#ifdef LOOP_BACK_TEST
	if (dev->id == 31){
		test_loopback_write(count);
	}
#endif

	ret = pdp_mux(dev, buf, count); // we should return only how much we wrote

	if (ret == 0)
	{
		ret = count;
	}

	return ret;
}

static int vs_write_room(struct tty_struct *tty)
{
	return 8192*2;		/* TODO: No idea where is number came from?? */
}

static int vs_chars_in_buffer(struct tty_struct *tty)
{
	return 0;
}

static int vs_ioctl(struct tty_struct *tty, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	return -ENOIOCTLCMD;
}

static struct tty_operations multipdp_tty_ops = {
	.open 		= vs_open,
	.close 		= vs_close,
	.write 		= vs_write,
	.write_room = vs_write_room,
	.ioctl 		= vs_ioctl,
	.chars_in_buffer = vs_chars_in_buffer,
};

static struct tty_driver* get_tty_driver_by_id(struct pdp_info *dev)
{
    int index = 0;

    switch (dev->id) {
        case 1:        index = 0;    break;
        case 8:        index = 1;    break;
        case 5:        index = 2;    break;
        case 6:        index = 3;    break;
        case 25:        index = 4;    break;
        case 30:        index = 5;    break;
        case 7:        index = 6;    break;
        case 9:        index = 7;    break;
        case 26:        index = 8;    break;
        case 27:        index = 9;    break;
        case 28:        index = 10;    break;
        case 29:        index = 11;    break;
        case 2:        index = 12;    break;
        case 31:        index = 13;    break;
        default:    index = 0;
    }

    return &dev->vs_dev.tty_driver[index];
}

static int get_minor_start_index(int id)
{
    int start = 0;

    switch (id) {
        case 1:        start = 0;    break;
        case 8:        start = 1;    break;
        case 5:        start = 2;    break;
        case 6:        start = 3;    break;
        case 25:        start = 4;    break;
        case 30:        start = 5;    break;
        case 7:        start = 6;    break;
        case 9:        start = 7;    break;
        case 26:        start = 8;    break;
        case 27:        start = 9;    break;
        case 28:        start = 10;    break;
        case 29:        start = 11;    break;
        case 2:        start = 12;    break;
        case 31:        start = 13;    break;
        default:    start = 0;
    }

    return start;
}

static int vs_add_dev(struct pdp_info *dev)
{
	struct tty_driver *tty_driver;

	tty_driver = get_tty_driver_by_id(dev);

	if (!tty_driver) {
		MULTIPDP_LOG_ERR("vs_add_dev ==> tty driver is NULL!\n");
		return -1;
	}

	kref_init(&tty_driver->kref);

	tty_driver->magic	= TTY_DRIVER_MAGIC;
	tty_driver->driver_name	= APP_DEVNAME;
	tty_driver->name	= dev->vs_dev.tty_name;
	tty_driver->major	= CSD_MAJOR_NUM;
	tty_driver->minor_start = get_minor_start_index(dev->id);
	tty_driver->num		= 1;
	tty_driver->type	= TTY_DRIVER_TYPE_SERIAL;
	tty_driver->subtype	= SERIAL_TYPE_NORMAL;
	tty_driver->flags	= TTY_DRIVER_REAL_RAW;
	tty_driver->ttys	= dev->vs_dev.tty_table; // 2.6 kernel porting
	tty_driver->termios	= dev->vs_dev.termios;
	tty_driver->termios_locked	= dev->vs_dev.termios_locked;

	tty_set_operations(tty_driver, &multipdp_tty_ops);
	return tty_register_driver(tty_driver);
}

static void vs_del_dev(struct pdp_info *dev)
{
	struct tty_driver *tty_driver = NULL;

	tty_driver = get_tty_driver_by_id(dev);
	tty_unregister_driver(tty_driver);

}

static inline struct pdp_info * pdp_get_dev(u8 id)
{
	int slot;

	for (slot = 0; slot < MAX_PDP_CONTEXT; slot++) {
		if (pdp_table[slot] && pdp_table[slot]->id == id) {
			return pdp_table[slot];
		}
	}
	return NULL;
}

static inline int pdp_add_dev(struct pdp_info *dev)
{
	int slot;

	if (pdp_get_dev(dev->id)) {
		MULTIPDP_LOG_ERR("[%s]: return -EBUSY \n",__FUNCTION__);
		return -EBUSY;
	}

	for (slot = 0; slot < MAX_PDP_CONTEXT; slot++) {
		if (pdp_table[slot] == NULL) {
			pdp_table[slot] = dev;
			return slot;
		}
	}
	MULTIPDP_LOG_ERR("[%s]: return -ENOSPC \n",__FUNCTION__);
	return -ENOSPC;
}

static inline struct pdp_info * pdp_remove_slot(int slot)
{
	struct pdp_info *dev;

	dev = pdp_table[slot];
	pdp_table[slot] = NULL;
	return dev;
}

static int pdp_activate(pdp_arg_t *pdp_arg, unsigned type, unsigned char intf_dev, unsigned flags )
{
	int ret = 0;
	struct pdp_info *dev = NULL;
	struct net_device *net = NULL;
	struct tty_driver * tty_driver = NULL;

	MULTIPDP_LOG_INFO(" %s \n",__FUNCTION__);
	MULTIPDP_LOG_INFO("pdp_activate ==> id: %d\n", pdp_arg->id);

//	dev = vmalloc(sizeof(struct pdp_info) + MAX_PDP_PACKET_LEN);
	dev = kmalloc(sizeof(struct pdp_info) + MAX_PDP_PACKET_LEN, GFP_KERNEL);
	if (dev == NULL) {
//		MULTIPDP_LOG_ERR("pdp_activate ==> vmalloc failed, out of memory\n");
		MULTIPDP_LOG_ERR("pdp_activate ==> kmalloc failed, out of memory\n");
		return -ENOMEM;
	}
	memset(dev, 0, sizeof(struct pdp_info));

	/* @LDK@ added by gykim on 20070203 for adjusting IPC 3.0 spec. */
	if (type == DEV_TYPE_NET) {
		dev->id = pdp_arg->id + g_adjust;
		DPRAM_LOG_ERR("%s, check!!! type: %d, dev->id: %d\n", __func__, type, dev->id);
	}
	else {
		dev->id = pdp_arg->id;
		dev->vs_dev.refcount = 0;
	}
	/* @LDK@ added by gykim on 20070203 for adjusting IPC 3.0 spec. */
	atomic_set(&dev->intf_dev, intf_dev);
	dev->type = type;
	dev->flags = flags;
	dev->tx_buf = (u8 *)(dev + 1);

	if (type == DEV_TYPE_NET) {

		MULTIPDP_LOG_INFO("calling vnet_add_dev\n");
		if(pdp_arg->en_interface_type == IPADDR_TYPE_IPV4)
			net = vnet_add_dev((void *)dev);
		else if((pdp_arg->en_interface_type == IPADDR_TYPE_IPV6) || (pdp_arg->en_interface_type == IPADDR_TYPE_IPV4V6))
			net = vnet_add_dev_v6((void *)dev);
		else {
			MULTIPDP_LOG_ERR("pdp_activate invalid argument!\n");
			//vfree(dev);
			kfree(dev);
			return -EINVAL;
		}

		if (net == NULL) {
			//vfree(dev);
			kfree(dev);
			return -ENOMEM;
		}

		dev->vn_dev.netq_init = 0;
		dev->vn_dev.netq_active = g_datastatus;
		init_MUTEX(&dev->vn_dev.netq_sem);

		dev->vn_dev.net = net;
		strcpy(pdp_arg->ifname, net->name);

		down(&pdp_lock);
		ret = pdp_add_dev(dev);
		if (ret < 0) {
			MULTIPDP_LOG_ERR("pdp_activate ==> pdp_add_dev() failed\n");
			up(&pdp_lock);
			vnet_del_dev(dev->vn_dev.net);
			//vfree(dev);
			kfree(dev);
			return ret;
		}
		up(&pdp_lock);
		//memcpy(net->int_iden, pdp_arg->int_iden, 8); /* added by SS in kernel. Removing the same.
		MULTIPDP_LOG_INFO("%s(id: %u) network device created\n", net->name, dev->id);
	} 
	else if (type == DEV_TYPE_SERIAL) {
		init_MUTEX(&dev->vs_dev.write_lock);
		strcpy(dev->vs_dev.tty_name, pdp_arg->ifname);

		ret = vs_add_dev(dev);
		if (ret < 0) {
			MULTIPDP_LOG_ERR("pdp_activate ==> vs_add_dev() failed\n");
			//vfree(dev);
			kfree(dev);
			return ret;
		}

		down(&pdp_lock);
		ret = pdp_add_dev(dev);
		if (ret < 0) {
			MULTIPDP_LOG_ERR("pdp_activate ==> pdp_add_dev() failed\n");
			up(&pdp_lock);
			vs_del_dev(dev);
			//vfree(dev);
			kfree(dev);
			return ret;
		}
		up(&pdp_lock);

		tty_driver = get_tty_driver_by_id(dev);
		MULTIPDP_LOG_INFO("%s(id: %u) serial device is created.\n", tty_driver->name, dev->id);

	}
	return 0;
}

static int pdp_deactivate(pdp_arg_t *pdp_arg, int force)
{
	struct pdp_info *dev = NULL;
	struct tty_driver * tty_driver = NULL;

	MULTIPDP_LOG_INFO("pdp_deactivate ==> id: %d\n", pdp_arg->id);

	down(&pdp_lock);

	if (pdp_arg->id == 1) {
		MULTIPDP_LOG_WARN("Channel ID is 1, we will remove the network device (pdp) of channel ID: %d.\n",
				pdp_arg->id /*+ g_adjust*/);
	}

	else {
		MULTIPDP_LOG_INFO("pdp_deactivate ==> Channel ID: %d\n", pdp_arg->id);
	}

	pdp_arg->id = pdp_arg->id /*+ g_adjust*/;
	//pdp_arg->id += PDP_ID_ADJUST;
	MULTIPDP_LOG_INFO("pdp_deactivate ==> ID is adjusted, new ID: %d\n", pdp_arg->id);

	dev = pdp_get_dev(pdp_arg->id);

	if (dev == NULL) {
		MULTIPDP_LOG_ERR("pdp_deactivate ==> error not found id: %u\n", pdp_arg->id);
		up(&pdp_lock);
		return -EINVAL;
	}
	if (!force && dev->flags & DEV_FLAG_STICKY) {
		MULTIPDP_LOG_ERR("pdp_deactivate ==> sticky id: %u\n", pdp_arg->id);
		up(&pdp_lock);
		return -EACCES;
	}

	pdp_remove_dev(pdp_arg->id);
	up(&pdp_lock);

	if (dev->type == DEV_TYPE_NET) {

		DPRAM_LOG_ERR("%s, check!!! dev->type: %d\n", __func__, dev->type);

		MULTIPDP_LOG_WARN("%s(id: %u) network device removed\n",
				dev->vn_dev.net->name, dev->id);
		vnet_del_dev(dev->vn_dev.net);
	} else if (dev->type == DEV_TYPE_SERIAL) {

		tty_driver = get_tty_driver_by_id(dev);
		MULTIPDP_LOG_WARN("%s(id: %u) serial device removed.\n", tty_driver->name, dev->id);

		vs_del_dev(dev);
	}

	//vfree(dev);
	kfree(dev);

	return 0;
}

static void pdp_cleanup(void)
{
	int slot;
	struct pdp_info *dev = NULL;
	struct tty_driver * tty_driver = NULL;

	down(&pdp_lock);
	for (slot = 0; slot < MAX_PDP_CONTEXT; slot++) {
		dev = pdp_remove_slot(slot);
		if (dev) {
			if (dev->type == DEV_TYPE_NET) {
				DPRAM_LOG_ERR("%s, check!!! dev->type: %d\n", __func__, dev->type);

				MULTIPDP_LOG_WARN("%s(id: %u) network device removed\n",
						dev->vn_dev.net->name, dev->id);
				vnet_del_dev(dev->vn_dev.net);
			} else if (dev->type == DEV_TYPE_SERIAL) {

				tty_driver = get_tty_driver_by_id(dev);
				MULTIPDP_LOG_WARN("%s(id: %u) serial device removed.\n", tty_driver->name, dev->id);

				vs_del_dev(dev);
			}
			vfree(dev);
		}
	}
	up(&pdp_lock);
}

static int pdp_adjust(const int adjust)
{
	g_adjust = adjust;
	MULTIPDP_LOG_INFO("pdp_adjust ==> adjusting value: %d\n", adjust);
	return 0;
}

static int multipdp_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned char *arg)
{
	int ret, adjust/* , radio,datastatus*/ ;
	pdp_arg_t pdp_arg;

	switch (cmd) {
		case HN_PDP_ACTIVATE:
			{
				MULTIPDP_LOG_INFO ("HN_PDP_ACTIVATE \n");
				if (copy_from_user(&pdp_arg, (void *)arg, sizeof(pdp_arg))) {
					MULTIPDP_LOG_INFO ("HN_PDP_ACTIVATE copy_from_user failed \n");
					return -EFAULT;
				}

				MULTIPDP_LOG_INFO ("HN_PDP_ACTIVATE calling pdp_activate with DPRAM_DEV \n");
				ret = pdp_activate(&pdp_arg, DEV_TYPE_SERIAL,DPRAM_DEV, 0);

				if (ret < 0) {
					MULTIPDP_LOG_INFO ("HN_PDP_ACTIVATE pdp_activate failed \n");
					return ret;
				}
			}
			return copy_to_user((void *)arg, &pdp_arg, sizeof(pdp_arg));

		case HN_PDP_DEACTIVATE:
			if (copy_from_user(&pdp_arg, (void *)arg, sizeof(pdp_arg)))
				return -EFAULT;
			return pdp_deactivate(&pdp_arg, 0);

		case HN_PDP_ADJUST:
			if (copy_from_user(&adjust, (void *)arg, sizeof (int)))
				return -EFAULT;
			return pdp_adjust(adjust);

		case HN_PDP_TXSTART:
			pdp_tx_flag = 0;
			return 0;

		case HN_PDP_TXSTOP:
			pdp_tx_flag = 1;
			return 0;

	}
	return -EINVAL;
}

static struct file_operations multipdp_fops = {
	.owner =	THIS_MODULE,
	.ioctl =	multipdp_ioctl,
	.llseek =	no_llseek,
};

static struct miscdevice multipdp_dev = {
	.minor =	132, //MISC_DYNAMIC_MINOR,
	.name =		APP_DEVNAME,
	.fops =		&multipdp_fops,
};

static inline struct pdp_info *pdp_get_serdev(const char *name)
{
	int slot;
	struct pdp_info *dev;

	for (slot = 0; slot < MAX_PDP_CONTEXT; slot++) {
		dev = pdp_table[slot];
		if (dev && dev->type == DEV_TYPE_SERIAL &&
				strcmp(name, dev->vs_dev.tty_name) == 0) {
			return dev;
		}
	}
	return NULL;
}

static inline struct pdp_info *pdp_remove_dev(u8 id)
{
	int slot;
	struct pdp_info *dev;
	for (slot = 0; slot < MAX_PDP_CONTEXT; slot++) {
		if (pdp_table[slot] && pdp_table[slot]->id == id) {
			dev = pdp_table[slot];
			pdp_table[slot] = NULL;
			return dev;
		}
	}
	return NULL;
}

static int pdp_mux(struct pdp_info *dev, const void *data, size_t len   )
{
    int i, ret;
    size_t nbytes;
    u8 *tx_buf;
    struct pdp_hdr *hdr;
    const u8 *buf;

    tx_buf = dev->tx_buf;
    hdr = (struct pdp_hdr *)(tx_buf + 1);
    buf = data;

    hdr->id = dev->id;
    hdr->control = 0;

    while (len)
    {
        if (len > MAX_PDP_DATA_LEN)
        {
            nbytes = MAX_PDP_DATA_LEN;
        }
        else
        {
            nbytes = len;
        }
        hdr->len = nbytes + sizeof(struct pdp_hdr);

        tx_buf[0] = 0x7f;
        
        memcpy(tx_buf + 1 + sizeof(struct pdp_hdr), buf,  nbytes);
        
        tx_buf[1 + hdr->len] = 0x7e;

// Only for debugging in case of 'ttyCDMA==7', 'ttyEFS==8'
//if (dev->id == 7 ||dev->id == 8){
//MULTIPDP_LOG_WARN("[%s] dev->id= %u \n", __FUNCTION__, dev->id);
//multipdp_debug_dump_write_buffer(tx_buf, (hdr->len + 2));
//}

	for(i =0; i< DPRAM_TX_RETRY; i++)
	{
		ret = dpram_write(&dpram_table[RAW_INDEX], tx_buf, hdr->len + 2);
		if( ret > 0 )
		{
			break;
		}
		udelay(200);
	}

	if ( i>= DPRAM_TX_RETRY) {
		MULTIPDP_LOG_ERR(" pdp_mux() failed: ret=%d\n", ret);
	}

	if (ret < 0)
	{
		MULTIPDP_LOG_ERR("write_to_dpram() failed: %d\n", ret);
		return ret;
	}
	buf += nbytes;
	len -= nbytes;
    }
    return 0;
}

static int multipdp_init(void)
{
	int i;

	pdp_arg_t pdp_args[NUM_PDP_CONTEXT] = {
		{ .id = 1, .ifname = "ttyCSD" },
		{ .id = 2, .ifname = "ttyFOTA" },
		{ .id = 5, .ifname = "ttyGPS" },
		{ .id = 6, .ifname = "ttyXTRA" },
		{ .id = 7, .ifname = "ttyCDMA" },
		{ .id = 8, .ifname = "ttyEFS" },
		{ .id = 9, .ifname = "ttyTRFB" },
		{ .id = 25, .ifname = "ttySMD" },
		{ .id = 26, .ifname = "ttyVTVD" },
		{ .id = 27, .ifname = "ttyVTAD" },
		{ .id = 28, .ifname = "ttyVTCTRL" },
		{ .id = 29, .ifname = "ttyVTENT" },
		{ .id = 30, .ifname = "ttyPCM" },
		{ .id = 31, .ifname = "ttyLOBK" },
	};

	/* create serial device for Circuit Switched Data */
	for (i = 0; i < NUM_PDP_CONTEXT; i++) {
		if (pdp_activate(&pdp_args[i], DEV_TYPE_SERIAL, DPRAM_DEV, DEV_FLAG_STICKY) < 0) {
			MULTIPDP_LOG_ERR("failed to create a serial device for %s\n", pdp_args[i].ifname);
		}
	}

	return 0;

}

static void multipdp_exit(void)
{
	/* remove app. interface device */
	misc_deregister(&multipdp_dev);

	/* clean up PDP context table */
	pdp_cleanup();
}

static void init_devices(void)
{
	int i;
	for (i = 0; i < MAX_INDEX; i++) {
		init_MUTEX(&dpram_table[i].serial.sem);
		dpram_table[i].serial.open_count = 0;
		dpram_table[i].serial.tty = NULL;
	}
}

static void init_hw_setting(void)
{

	/* initial pin settings - dpram driver control */
	s3c_gpio_cfgpin(GPIO_DPRAM_INT_N, S3C_GPIO_SFN(GPIO_DPRAM_INT_N_AF));
	s3c_gpio_setpull(GPIO_DPRAM_INT_N, S3C_GPIO_PULL_NONE);
	set_irq_type(IRQ_DPRAM_INT_N, IRQ_TYPE_EDGE_FALLING);

}

static void kill_tasklets(void)
{

	tasklet_kill(&fmt_res_ack_tasklet);
	tasklet_kill(&raw_res_ack_tasklet);
	tasklet_kill(&fmt_send_tasklet);
	tasklet_kill(&raw_send_tasklet);

#if defined(USE_INTERRUPTABLE_LOAD)
	tasklet_kill(&interruptable_load_tasklet);
#endif

}

static int register_interrupt_handler(void)
{

	unsigned int dpram_irq/*, phone_active_irq*/;
	int retval = 0;

	/* @LDK@ interrupt area read - pin level will be driven high. */
	/* dpram_clear(); */

	/* @LDK@ dpram interrupt */
	dpram_irq = gpio_to_irq(GPIO_DPRAM_INT_N);
	retval = request_irq(dpram_irq, dpram_irq_handler, 
		IRQF_DISABLED|IRQF_TRIGGER_FALLING, 
		"dpram irq", NULL);

	if (retval) {
		DPRAM_LOG_ERR("request_irq() for dpram_irq is failed. [retval]: %d \n", retval);
//		unregister_dpram_driver();
		return -1;
	}

	retval = enable_irq_wake(dpram_irq);
	if (retval < 0)
	{
		DPRAM_LOG_ERR("enable_irq_wake() for dpram_irq is failed. [retval]: %d \n", retval);
		free_irq(dpram_irq, NULL);
		return -1;
	}

	Gdpram_irq=dpram_irq;

	return 0;
}

static void check_miss_interrupt(void)
{
	unsigned long flags;
	if (gpio_get_value(GPIO_PHONE_ACTIVE) &&
			(!gpio_get_value(GPIO_DPRAM_INT_N))) {
		DPRAM_LOG_ERR("there is a missed interrupt. try to read it!\n");

		local_irq_save(flags);
		dpram_irq_handler(Gdpram_irq, NULL);
		local_irq_restore(flags);
	}
}

static int dpram_suspend(struct platform_device *dev, pm_message_t state)
{

	DPRAM_LOG_INFO("[%s] opend\n", __func__);

	gpio_set_value(GPIO_PDA_ACTIVE, GPIO_LEVEL_LOW);
	return 0;

}

static int dpram_resume(struct platform_device *dev)
{

	DPRAM_LOG_INFO("[%s] opend\n", __func__);

	gpio_set_value(GPIO_PDA_ACTIVE, GPIO_LEVEL_HIGH);
	check_miss_interrupt();
	return 0;

}

static int __devinit dpram_probe(struct platform_device *pdev)
{
	int retval;

	printk(KERN_ERR "[DPRAM] *** Entering dpram_probe()\n");

	/* @LDK@ register dpram (tty) driver */
	retval = register_dpram_driver();
	if (retval) {
		printk(KERN_ERR  "Failed to register dpram (tty) driver.\n");
		return -1;
	}

#ifdef _ENABLE_ERROR_DEVICE
	/* @LDK@ register dpram error device */
	retval = register_dpram_err_device();
	if (retval) {
		printk(KERN_ERR  "Failed to register dpram error device.\n");
		unregister_dpram_driver();
		return -1;
	}

	memset((void *)dpram_err_buf, '\0', sizeof dpram_err_buf);
#endif /* _ENABLE_ERROR_DEVICE */

	retval = misc_register(&multipdp_dev);    /* create app. interface device */
	if (retval < 0)
	{
		printk(KERN_ERR  "misc_register() failed\n");
		return -1;
	}

	multipdp_init();

	/* @LDK@ H/W setting */
	init_hw_setting();
	dpram_shared_bank_remap();
	
	/* @LDK@ initialize device table */
	init_devices();

	atomic_set(&dpram_write_lock, 0);
	atomic_set(&raw_txq_req_ack_rcvd, 0);
	DPRAM_LOG_INFO("%s, raw_txq_req_ack_rcvd: 0x%x\n", __func__, raw_txq_req_ack_rcvd);
	atomic_set(&fmt_txq_req_ack_rcvd, 0);
	DPRAM_LOG_INFO("%s, fmt_txq_req_ack_rcvd: 0x%x\n", __func__, fmt_txq_req_ack_rcvd);

	/* @LDK@ register interrupt handler */
//	retval = register_interrupt_handler();
//	if (retval < 0) {
//		DPRAM_LOG_ERR("[DPRAM] *** dpram_probe() failed to register interrupt handler.\n");
//		return -1;
//	}

	dpram_platform_init();

#if defined(USE_INTERRUPTABLE_LOAD)
	check_param.total_size = 0;
	check_param.rest_size = 0;
	check_param.send_size = 0;
	check_param.copy_start = 0;
	check_param.copy_complete = 0;
	check_param.boot_complete = 0;
#endif

#ifdef CONFIG_PROC_FS
	create_proc_read_entry(DRIVER_PROC_ENTRY, 0, 0, dpram_read_proc, NULL);
#endif	/* CONFIG_PROC_FS */

	/* @LDK@ check out missing interrupt from the phone */
	/* check_miss_interrupt(); */

#ifdef _ENABLE_DEBUG_PRINTS
	register_dpram_debug_control_attribute();
	register_multipdp_debug_control_attribute();
#endif

	retval = device_create_file(multipdp_dev.this_device, &dev_attr_debug);

#if defined(USE_INTERRUPTABLE_LOAD)
	// Clear PHONE2PDA Interrupt 
	ClearPendingInterruptFromModem();

	/* @LDK@ register interrupt handler */
	retval = register_interrupt_handler();
	if (retval < 0) {
		DPRAM_LOG_ERR("[DPRAM] *** dpram_probe() failed to register interrupt handler.\n");
		unregister_dpram_driver();
#ifdef _ENABLE_ERROR_DEVICE
		unregister_dpram_err_device();
#endif		
		return -1;
	}
#endif /* USE_INTERRUPTABLE_LOAD */

	printk(KERN_ERR  "[DPRAM] *** Leaving dpram_probe()\n");
	return 0;
}

static int __devexit dpram_remove(struct platform_device *dev)
{

#ifdef _ENABLE_DEBUG_PRINTS
	deregister_dpram_debug_control_attribute();
	deregister_multipdp_debug_control_attribute();
#endif

	/* @LDK@ unregister dpram (tty) driver */
	unregister_dpram_driver();

	/* @LDK@ unregister dpram error device */
#ifdef _ENABLE_ERROR_DEVICE
	unregister_dpram_err_device();
#endif

	/* remove app. interface device */
	multipdp_exit();

	/* @LDK@ unregister irq handler */
	free_irq(Gdpram_irq, NULL);

	kill_tasklets();

	return 0;
}

static struct platform_driver platform_dpram_driver = {
	.probe		= dpram_probe,
	.remove		= __devexit_p(dpram_remove),
	.suspend	= dpram_suspend,
	.resume 	= dpram_resume,
	.driver	= {
		.name	= "dpram-device",
	},
};

/* init & cleanup. */
static int __init dpram_init(void)
{
	wake_lock_init(&dpram_wake_lock, WAKE_LOCK_SUSPEND, "DPRAM");
	return platform_driver_register(&platform_dpram_driver);
}
static void __exit dpram_exit(void)
{
	wake_lock_destroy(&dpram_wake_lock);
	platform_driver_unregister(&platform_dpram_driver);
}

void ClearPendingInterruptFromModem(void)
{
	u16 in_interrupt = 0;
	READ_FROM_DPRAM((void *)&in_interrupt, DPRAM_PHONE2PDA_INTERRUPT_ADDRESS, sizeof(in_interrupt));
}

void dpram_debug_dump_raw_read_buffer(const unsigned char  *buf, int len)
{
#ifdef PRINT_READ
	int i = 0;

//	MULTIPDP_LOG_READ("[MULTIPDP] RAW READ:\n");
	if(len<16)
	{
		for (i = 0; i < len; i++)
			MULTIPDP_LOG_READ( "IPC(RR):%02x ", *((unsigned char *)buf + i));
	}
	else
	{
		MULTIPDP_LOG_READ(
		"IPC(RR):%02X %02X %02X %02x  %02X %02X %02X %02X  "
		"%02X %02X %02X %02X  %02X %02X %02X %02X\n",
		*(buf+0), *(buf+1), *(buf+2), *(buf+3), 
		*(buf+4), *(buf+5), *(buf+6), *(buf+7), 
		*(buf+8), *(buf+9), *(buf+10), *(buf+11), 
		*(buf+12), *(buf+13), *(buf+14), *(buf+15)
		);
	}

//	MULTIPDP_LOG_READ("\n");
#endif
}

void dpram_debug_dump_write_buffer(const unsigned char  *buf, int len)
{
#ifdef PRINT_WRITE
	int i = 0;

//	DPRAM_LOG_WRITE("[DPRAM] WRITE:\n");
	if(len<16)
	{
		for (i = 0; i < len; i++)
			DPRAM_LOG_WRITE( "IPC(FW):%02x ", *((unsigned char *)buf + i));
	}
	else
	{
		DPRAM_LOG_WRITE(
		"IPC(FW):%02X %02X %02X %02x  %02X %02X %02X %02X  "
		"%02X %02X %02X %02X  %02X %02X %02X %02X\n",
		*(buf+0), *(buf+1), *(buf+2), *(buf+3), 
		*(buf+4), *(buf+5), *(buf+6), *(buf+7), 
		*(buf+8), *(buf+9), *(buf+10), *(buf+11), 
		*(buf+12), *(buf+13), *(buf+14), *(buf+15)
		);
	}

//	DPRAM_LOG_WRITE("\n");
#endif
}

void multipdp_debug_dump_write_buffer(const unsigned char  *buf, int len)
{
#ifdef MULTIPDP_PRINT_WRITE
	int i = 0;

//	MULTIPDP_LOG_WRITE("[MULTIPDP] WRITE:\n");
	if(len<16)
	{
		for (i = 0; i < len; i++)
			MULTIPDP_LOG_WRITE( "IPC(RW):%02x ", *((unsigned char *)buf + i));
	}
	else
	{
		MULTIPDP_LOG_WRITE(
		"IPC(RW):%02X %02X %02X %02x  %02X %02X %02X %02X  "
		"%02X %02X %02X %02X  %02X %02X %02X %02X\n",
		*(buf+0), *(buf+1), *(buf+2), *(buf+3), 
		*(buf+4), *(buf+5), *(buf+6), *(buf+7), 
		*(buf+8), *(buf+9), *(buf+10), *(buf+11), 
		*(buf+12), *(buf+13), *(buf+14), *(buf+15)
		);
	}

//	MULTIPDP_LOG_WRITE("\n");
#endif
}

#ifdef _ENABLE_DEBUG_PRINTS

#define CBUF_SIZE 1024
static char cbuf[CBUF_SIZE] = {'.'};
static char tcbuf[CBUF_SIZE] = {'.'};;
static int cbuf_idx;

void dpram_debug_print_cbuf(char *strptr)
{
	int len, len1, len2;
	len = strlen(strptr);

	if (len > CBUF_SIZE) {
		printk(KERN_ERR "[DPRAM] ILLEGAL STRING\n");
		return;
	}

	if ((cbuf_idx + len) < CBUF_SIZE) {
		memcpy(&cbuf[cbuf_idx], strptr, len);
		cbuf_idx += len;
		if (cbuf_idx >= CBUF_SIZE)
			cbuf_idx = 0;
	} else {
		len1 = CBUF_SIZE - cbuf_idx;
		len2 = len - len1;
		memcpy(&cbuf[cbuf_idx], strptr, len1);
		strptr += len1;
		memcpy(cbuf, strptr, len2);
		cbuf_idx = len2;
	}
}


void dpram_debug_cbuf_reinit(void)
{
	int len;
	len = CBUF_SIZE - cbuf_idx;
	memcpy(tcbuf, &cbuf[cbuf_idx], len);
	memcpy(&tcbuf[len], cbuf, (CBUF_SIZE - len));
	tcbuf[CBUF_SIZE - 1] = 0;
	printk(KERN_ERR "%s\nComplete print\n", tcbuf);
	memset(cbuf, '.', CBUF_SIZE);
	cbuf_idx = 0;
}

static ssize_t dpram_store_debug_level(struct device_driver *ddp, const char *buf, size_t count)
{
	u16 value;
	char *after;
	value = simple_strtoul(buf, &after, 10);
	printk(KERN_ERR "[%s] value = %x\n", __func__, value);
	dpram_debug_mask = value;
	dpram_debug_mask |= (DPRAM_PRINT_ERROR | DPRAM_PRINT_WARNING); /* ERROR/WARN are always enabled */
	dpram_debug_cbuf_reinit();

	/* dpram_test_set_head_tail(); */
	return 1;
}
static ssize_t dpram_show_debug_level(struct device_driver *ddp, char *buf)
{
	printk(KERN_ERR "%s, dpram_debug_mask %x\n", __func__, dpram_debug_mask);
	return snprintf(buf, PAGE_SIZE, "%d\n", dpram_debug_mask);
}

struct driver_attribute debug_ctrl_attr;


void register_dpram_debug_control_attribute(void)
{
	int retval = 0;

	debug_ctrl_attr.attr.name = "debugctrl";
	debug_ctrl_attr.attr.owner = platform_dpram_driver.driver.owner;
	debug_ctrl_attr.attr.mode = S_IRUSR | S_IWUSR;
	debug_ctrl_attr.show = dpram_show_debug_level;
	debug_ctrl_attr.store = dpram_store_debug_level;

	cbuf_idx = 0;
	memset(cbuf, '.', CBUF_SIZE);

	retval = driver_create_file(&platform_dpram_driver.driver, &debug_ctrl_attr);
}

void deregister_dpram_debug_control_attribute(void)
{
	driver_remove_file(&platform_dpram_driver.driver, &debug_ctrl_attr);
}

static char s_buf[1024];

void  dpram_debug_print(u8 print_prefix, u32 mask,  const char *fmt, ...)
{
	va_list args;

	if (dpram_debug_mask & mask) {

		va_start(args, fmt);
		vsprintf(s_buf, fmt, args);
		va_end(args);

		if (dpram_debug_mask & DPRAM_PRINT_CBUF) {
			dpram_debug_print_cbuf(s_buf);
		} else {
			if (print_prefix)
				printk(KERN_ERR "[DPRAM]");
			printk(s_buf);
		}
	}
}

static ssize_t mulitpdp_store_debug_level(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t len)
{
	u16 value;
	char *after;
	value = simple_strtoul(buf, &after, 10);
	printk("mulitpdp_store_debug_level, value = %x\n", value);
	mulitpdp_debug_mask = value;
	mulitpdp_debug_mask |= (MULTIPDP_PRINT_ERROR | MULTIPDP_PRINT_WARNING); // ERROR/WARN are always enabled
	printk("mulitpdp_debug_mask = %x\n", mulitpdp_debug_mask);
	return 1;
}
static ssize_t mulitpdp_show_debug_level(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("mulitpdp_show_debug_level, mulitpdp_debug_mask %x\n", mulitpdp_debug_mask);
	return snprintf(buf, PAGE_SIZE, "%d\n", mulitpdp_debug_mask);
}

static DEVICE_ATTR(multipdp_debug, S_IRUGO | S_IWUSR, mulitpdp_show_debug_level, mulitpdp_store_debug_level);
void register_multipdp_debug_control_attribute(void)
{
	int retval = 0;

	retval = device_create_file(multipdp_dev.this_device, &dev_attr_multipdp_debug);

}
void deregister_multipdp_debug_control_attribute(void)
{
	device_remove_file(multipdp_dev.this_device, &dev_attr_multipdp_debug);
}
void multipdp_debug_print(u32 mask,  const char *fmt, ...)
{
	if (mulitpdp_debug_mask & mask)
	{
		static char s_buf[1024];
		va_list args;

		va_start(args, fmt);
		vsprintf(s_buf, fmt, args);
		va_end(args);

		//printk("[MULTIPDP]");
		printk(KERN_ERR "[DPRAM]");
		printk(s_buf);
	}
}
#endif /*_ENABLE_DEBUG_PRINTS*/

module_init(dpram_init);
module_exit(dpram_exit);

MODULE_AUTHOR("SAMSUNG ELECTRONICS CO., LTD");

MODULE_DESCRIPTION("DPRAM Device Driver.");

MODULE_LICENSE("GPL");
