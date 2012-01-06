#include <linux/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <mach/regs-gpio.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
#include <linux/modemctl.h>
#if defined(CONFIG_SAMSUNG_PHONE_MODEMCTL)
#include <linux/irq.h>
#endif
#if defined(CONFIG_MACH_C1_KOR_LGT)
#include <linux/mfd/max8997.h>
#endif

#if !defined(CONFIG_SAMSUNG_PHONE_MODEMCTL)
static struct modemctl_platform_data mdmctl_data;
void modemctl_cfg_gpio(void)
{
	int err = 0;

	unsigned gpio_phone_on = mdmctl_data.gpio_phone_on;
	unsigned gpio_phone_active = mdmctl_data.gpio_phone_active;
	unsigned gpio_cp_rst = mdmctl_data.gpio_cp_reset;
	unsigned gpio_pda_active = mdmctl_data.gpio_pda_active;
	unsigned gpio_cp_req_reset = mdmctl_data.gpio_cp_req_reset;
	unsigned gpio_ipc_slave_wakeup = mdmctl_data.gpio_ipc_slave_wakeup;
	unsigned gpio_ipc_host_wakeup = mdmctl_data.gpio_ipc_host_wakeup;
	unsigned gpio_suspend_request = mdmctl_data.gpio_suspend_request;
	unsigned gpio_active_state = mdmctl_data.gpio_active_state;
	unsigned gpio_cp_dump_int = mdmctl_data.gpio_cp_dump_int;

	/*TODO: check uart init func AP FLM BOOT RX -- */
	s3c_gpio_setpull(S5PV310_GPA1(4), S3C_GPIO_PULL_UP);

	err = gpio_request(gpio_phone_on, "PHONE_ON");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "PHONE_ON");
	} else {
		gpio_direction_output(gpio_phone_on, 0);
		s3c_gpio_setpull(gpio_phone_on, S3C_GPIO_PULL_NONE);
	}
	err = gpio_request(gpio_cp_rst, "CP_RST");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "CP_RST");
	} else {
		gpio_direction_output(gpio_cp_rst, 0);
		s3c_gpio_setpull(gpio_cp_rst, S3C_GPIO_PULL_NONE);
	}
	err = gpio_request(gpio_pda_active, "PDA_ACTIVE");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "PDA_ACTIVE");
	} else {
		gpio_direction_output(gpio_pda_active, 0);
		s3c_gpio_setpull(gpio_pda_active, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_cp_req_reset, "CP_REQ_RESET");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "CP_REQ_RESET");
	} else {
		gpio_direction_output(gpio_cp_req_reset, 0);
		s3c_gpio_setpull(gpio_cp_req_reset, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_ipc_slave_wakeup, "IPC_SLAVE_WAKEUP");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n",
			"IPC_SLAVE_WAKEUP");
	} else {
		gpio_direction_output(gpio_ipc_slave_wakeup, 0);
		s3c_gpio_setpull(gpio_ipc_slave_wakeup, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_ipc_host_wakeup, "IPC_HOST_WAKEUP");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "IPC_HOST_WAKEUP");
	} else {
		gpio_direction_output(gpio_ipc_host_wakeup, 0);
		s3c_gpio_cfgpin(gpio_ipc_host_wakeup, S3C_GPIO_SFN(0xF));
		s3c_gpio_setpull(gpio_ipc_host_wakeup, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_suspend_request, "SUSPEND_REQUEST");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "SUSPEND_REQUEST");
	} else {
		gpio_direction_input(gpio_suspend_request);
		s3c_gpio_setpull(gpio_suspend_request, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_active_state, "ACTIVE_STATE");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "ACTIVE_STATE");
	} else {
		gpio_direction_output(gpio_active_state, 0);
		s3c_gpio_setpull(gpio_active_state, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_phone_active, "PHONE_ACTIVE");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "PHONE_ACTIVE");
	} else {
		gpio_direction_output(gpio_phone_active, 0);
		s3c_gpio_cfgpin(gpio_phone_active, S3C_GPIO_SFN(0xF));
		s3c_gpio_setpull(gpio_phone_active, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_cp_dump_int, "CP_DUMP_INT");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "CP_DUMP_INT");
	} else {
		gpio_direction_output(gpio_cp_dump_int, 0);
		s3c_gpio_cfgpin(gpio_cp_dump_int, S3C_GPIO_SFN(0xF));
		s3c_gpio_setpull(gpio_cp_dump_int, S3C_GPIO_PULL_DOWN);
	}
}

static void xmm6260_vcc_init(struct modemctl *mc)
{
	int err;

	if (!mc->vcc) {
		mc->vcc = regulator_get(NULL, "vhsic");
		if (IS_ERR(mc->vcc)) {
			err = PTR_ERR(mc->vcc);
			dev_dbg(mc->dev, "No VHSIC_1.2V regualtor: %d\n", err);
			mc->vcc = NULL;
		}
	}

	if (mc->vcc)
		regulator_enable(mc->vcc);
}

static void xmm6260_vcc_off(struct modemctl *mc)
{
	if (mc->vcc)
		regulator_disable(mc->vcc);
}

static void xmm6260_on(struct modemctl *mc)
{
	dev_dbg(mc->dev, "%s\n", __func__);
	if (!mc->gpio_cp_reset || !mc->gpio_phone_on || !mc->gpio_cp_req_reset)
		return;

	xmm6260_vcc_init(mc);

	gpio_set_value(mc->gpio_phone_on, 0);
	gpio_set_value(mc->gpio_cp_reset, 0);
	udelay(160);

	gpio_set_value(mc->gpio_pda_active, 0);
	gpio_set_value(mc->gpio_active_state, 0);
	msleep(100);

	gpio_set_value(mc->gpio_cp_reset, 1);
	udelay(160);
	gpio_set_value(mc->gpio_cp_req_reset, 1);
	udelay(160);

	gpio_set_value(mc->gpio_phone_on, 1);

	msleep(20);
	gpio_set_value(mc->gpio_active_state, 1);
	gpio_set_value(mc->gpio_pda_active, 1);
}

static void xmm6260_off(struct modemctl *mc)
{
	dev_dbg(mc->dev, "%s\n", __func__);
	if (!mc->gpio_cp_reset || !mc->gpio_phone_on)
		return;

	gpio_set_value(mc->gpio_phone_on, 0);
	gpio_set_value(mc->gpio_cp_reset, 0);

	xmm6260_vcc_off(mc);
}

static void xmm6260_reset(struct modemctl *mc)
{
	dev_dbg(mc->dev, "%s\n", __func__);
	if (!mc->gpio_cp_reset || !mc->gpio_cp_req_reset)
		return;

/*	gpio_set_value(mc->gpio_pda_active, 0);
	gpio_set_value(mc->gpio_active_state, 0);*/
	gpio_set_value(mc->gpio_cp_reset, 0);
	gpio_set_value(mc->gpio_cp_req_reset, 0);

	msleep(100);

	gpio_set_value(mc->gpio_cp_reset, 1);
	udelay(160);
	gpio_set_value(mc->gpio_cp_req_reset, 1);
}

/* move the PDA_ACTIVE Pin control to sleep_gpio_table */
static void xmm6260_suspend(struct modemctl *mc)
{
	xmm6260_vcc_off(mc);
}

static void xmm6260_resume(struct modemctl *mc)
{
	xmm6260_vcc_init(mc);
}

static struct modemctl_platform_data mdmctl_data = {
	.name = "xmm6260",
	.gpio_phone_on = GPIO_PHONE_ON,
	.gpio_phone_active = GPIO_PHONE_ACTIVE,
	.gpio_pda_active = GPIO_PDA_ACTIVE,
	.gpio_cp_reset = GPIO_CP_RST,
	.gpio_cp_req_reset = GPIO_CP_REQ_RESET,
	.gpio_ipc_slave_wakeup = GPIO_IPC_SLAVE_WAKEUP,
	.gpio_ipc_host_wakeup = GPIO_IPC_HOST_WAKEUP,
	.gpio_suspend_request = GPIO_SUSPEND_REQUEST,
	.gpio_active_state = GPIO_ACTIVE_STATE,
	.gpio_cp_dump_int = GPIO_CP_DUMP_INT,
	.ops = {
		.modem_on = xmm6260_on,
		.modem_off = xmm6260_off,
		.modem_reset = xmm6260_reset,
		.modem_suspend = xmm6260_suspend,
		.modem_resume = xmm6260_resume,
		.modem_cfg_gpio = modemctl_cfg_gpio,
	}
};

/* TODO: check the IRQs..... */
static struct resource mdmctl_res[] = {
	[0] = {
		.start = IRQ_PHONE_ACTIVE,
		.end = IRQ_PHONE_ACTIVE,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device modemctl = {
	.name = "modemctl",
	.id = -1,
	.num_resources = ARRAY_SIZE(mdmctl_res),
	.resource = mdmctl_res,

	.dev = {
		.platform_data = &mdmctl_data,
	},
};

#else
static struct modemctl_platform_data mdmctl_data;
void modemctl_cfg_gpio(void)
{
	int err = 0;
	
	unsigned gpio_phone_on = mdmctl_data.gpio_phone_on;
	unsigned gpio_phone_active = mdmctl_data.gpio_phone_active;
	unsigned gpio_cp_rst = mdmctl_data.gpio_cp_reset;
	unsigned gpio_cp_rst_msm = mdmctl_data.gpio_cp_reset_msm;
	unsigned gpio_pda_active = mdmctl_data.gpio_pda_active;
	unsigned gpio_boot_sw_sel = mdmctl_data.gpio_boot_sw_sel;
//	unsigned gpio_cp_dump_int = mdmctl_data.gpio_cp_dump_int;

	err = gpio_request(gpio_phone_on, "PHONE_ON");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "PHONE_ON");
	} else {
		gpio_direction_output(gpio_phone_on, GPIO_LEVEL_LOW);
		s3c_gpio_setpull(gpio_phone_on, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_cp_rst, "CP_RST");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "CP_RST");
	} else {
		gpio_direction_output(gpio_cp_rst, GPIO_LEVEL_LOW);
		s3c_gpio_setpull(gpio_cp_rst, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_cp_rst_msm, "CP_RST_MSM");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "CP_RST_MSM");
	} else {
		gpio_direction_output(gpio_cp_rst_msm, GPIO_LEVEL_LOW);
		s3c_gpio_cfgpin(gpio_cp_rst_msm, S3C_GPIO_SFN(0x1));	// output
		s3c_gpio_setpull(gpio_cp_rst_msm, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_pda_active, "PDA_ACTIVE");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "PDA_ACTIVE");
	} else {
		gpio_direction_output(gpio_pda_active, GPIO_LEVEL_LOW);
		s3c_gpio_setpull(gpio_pda_active, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_boot_sw_sel, "BOOT_SW_SEL");   // p1_prime
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "BOOT_SW_SEL");
	} else {
		gpio_direction_output(gpio_boot_sw_sel, GPIO_LEVEL_LOW);
		s3c_gpio_setpull(gpio_boot_sw_sel, S3C_GPIO_PULL_NONE);
	}	

	err = gpio_request(gpio_phone_active, "PHONE_ACTIVE");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "PHONE_ACTIVE");
	} else {
		gpio_direction_output(gpio_phone_active, GPIO_LEVEL_LOW);
		s3c_gpio_cfgpin(gpio_phone_active, S3C_GPIO_SFN(0xF));
		s3c_gpio_setpull(gpio_phone_active, S3C_GPIO_PULL_NONE);
	}

//	err = gpio_request(gpio_cp_dump_int, "CP_DUMP_INT");
//	if (err) {
//		printk(KERN_ERR "fail to request gpio %s\n", "CP_DUMP_INT");
//	} else {
//		gpio_direction_output(gpio_cp_dump_int, 0);
//		s3c_gpio_cfgpin(gpio_cp_dump_int, S3C_GPIO_SFN(0xF));
//		s3c_gpio_setpull(gpio_cp_dump_int, S3C_GPIO_PULL_NONE);
//	}
}

static void msm_vbus_on(struct modemctl *mc)
{
	int err;

	if (system_rev >= 0x06) {
#ifdef GPIO_USB_BOOT_EN
		dev_info(mc->dev, "%s : set USB_BOOT_EN\n", __func__);
		gpio_request(GPIO_USB_BOOT_EN, "USB_BOOT_EN");
		gpio_direction_output(GPIO_USB_BOOT_EN,1);
		gpio_free(GPIO_USB_BOOT_EN);
#endif
	}
	else {
#ifdef GPIO_USB_OTG_EN
		gpio_request(GPIO_USB_OTG_EN, "USB_OTG_EN");
		gpio_direction_output(GPIO_USB_OTG_EN,1);
		gpio_free(GPIO_USB_OTG_EN);
#endif
	}
	mdelay(10);

	if (!mc->cp_vbus) {
		mc->cp_vbus = regulator_get(NULL, "safeout2");
		if (IS_ERR(mc->cp_vbus)) {
			err = PTR_ERR(mc->cp_vbus);
			printk(KERN_ERR "No CP_VBUS_4.9(safeout2) regualtor: %d\n", err);
			mc->cp_vbus = NULL;
		}
	}

	if (mc->cp_vbus) {
		dev_info(mc->dev, "%s\n", __func__);
		regulator_enable(mc->cp_vbus);
	}
}

static void msm_vbus_off(struct modemctl *mc)
{
	if (mc->cp_vbus) {
		dev_info(mc->dev, "%s\n", __func__);
		regulator_disable(mc->cp_vbus);
	}

	if (system_rev >= 0x06) {
#ifdef GPIO_USB_BOOT_EN
		gpio_request(GPIO_USB_BOOT_EN, "USB_BOOT_EN");
		gpio_direction_output(GPIO_USB_BOOT_EN,0);
		gpio_free(GPIO_USB_BOOT_EN);
#endif
	}
	else {
#ifdef GPIO_USB_OTG_EN
		gpio_request(GPIO_USB_OTG_EN, "USB_OTG_EN");
		gpio_direction_output(GPIO_USB_OTG_EN,0);
		gpio_free(GPIO_USB_OTG_EN);
#endif
	}

}

static void msm_on(struct modemctl *mc)
{
	dev_dbg(mc->dev, "%s\n", __func__);
	if(!mc->gpio_cp_reset || !mc->gpio_cp_reset_msm || !mc->gpio_phone_on)
		return;

	gpio_set_value(mc->gpio_pda_active, 0);
	//gpio_set_value(mc->gpio_phone_on, 0);
	//gpio_set_value(mc->gpio_cp_reset, 0);
	//msleep(500);
	gpio_set_value(mc->gpio_cp_reset, 1);
	gpio_set_value(mc->gpio_cp_reset_msm, 1);
	msleep(30);
	gpio_set_value(mc->gpio_phone_on, 1);
	//msleep(30);
	msleep(300);
	gpio_set_value(mc->gpio_phone_on, 0);
	msleep(500);

	gpio_set_value(mc->gpio_pda_active, 1);

}

static void msm_off(struct modemctl *mc)
{
	dev_dbg(mc->dev, "%s\n", __func__);
	if(!mc->gpio_cp_reset || !mc->gpio_cp_reset_msm || !mc->gpio_phone_on)
		return;

	gpio_set_value(mc->gpio_phone_on, 0);
	gpio_set_value(mc->gpio_cp_reset, 0);
	gpio_set_value(mc->gpio_cp_reset_msm, 0);
}

static void msm_reset(struct modemctl *mc)
{
	dev_dbg(mc->dev, "%s\n", __func__);
	if(!mc->gpio_cp_reset || !mc->gpio_cp_reset_msm || !mc->gpio_phone_on)
		return;

	/* To Do :
	 * hard_reset(RESET_PMU_N) and soft_reset(RESET_REQ_N)
	 * should be divided later.
	 * soft_reset is used for CORE_DUMP
	 */
	if(system_rev >= 0x05)
	{
		dev_err(mc->dev, "[%s] system_rev: %d\n", __func__, system_rev);

		gpio_set_value(mc->gpio_cp_reset_msm, 0);
		msleep(100); /* no spec, confirm later exactly how much time
				   needed to initialize CP with RESET_PMU_N */
		gpio_set_value(mc->gpio_cp_reset_msm, 1);
		msleep(40); /* > 37.2 + 2 msec */
	}
	else
	{
		dev_err(mc->dev, "[%s] system_rev: %d\n", __func__, system_rev);

		gpio_set_value(mc->gpio_cp_reset, 0);
		msleep(500); /* no spec, confirm later exactly how much time
				   needed to initialize CP with RESET_PMU_N */
		gpio_set_value(mc->gpio_cp_reset, 1);
		msleep(40); /* > 37.2 + 2 msec */
	}
//	gpio_set_value(mc->gpio_phone_on, 0);
//	gpio_set_value(mc->gpio_cp_reset, 0);
//	gpio_set_value(mc->gpio_cp_reset_msm, 0);

}

static void msm_suspend(struct modemctl *mc)
{
	//TBD
}

static void msm_resume(struct modemctl *mc)
{
	//TBD
}

static void msm_boot_on(struct modemctl *mc)
{
	dev_dbg(mc->dev, "%s\n", __func__);
	msm_vbus_on(mc);

	if(mc->gpio_boot_sw_sel)
		gpio_set_value(mc->gpio_boot_sw_sel, 0);
	mc->usb_boot = true;
}

static void msm_boot_off(struct modemctl *mc)
{
	dev_dbg(mc->dev, "%s\n", __func__);
	msm_vbus_off(mc);

	if(mc->gpio_boot_sw_sel)
		gpio_set_value(mc->gpio_boot_sw_sel,1);
	mc->usb_boot = false;

#if defined(CONFIG_MACH_C1_KOR_LGT)
	max8997_muic_update_status(3000);
#endif
}

/* Modem control */
static struct modemctl_platform_data mdmctl_data = {
	.name = "msm", // QC : "msm", Infinion : "xmm"
	.gpio_phone_on = GPIO_PHONE_ON,
	.gpio_phone_active = GPIO_PHONE_ACTIVE,
	.gpio_pda_active = GPIO_PDA_ACTIVE,
	.gpio_cp_reset = GPIO_CP_RST,
	.gpio_cp_reset_msm = GPIO_CP_RST_MSM,
	.gpio_boot_sw_sel = GPIO_BOOT_SW_SEL,  // p1_prime
//	.gpio_cp_dump_int = GPIO_CP_DUMP_INT,
	.ops = {
		.modem_on = msm_on,
		.modem_off = msm_off,
		.modem_reset = msm_reset,
		.modem_suspend = msm_suspend,
		.modem_resume = msm_resume,
		.modem_cfg_gpio = modemctl_cfg_gpio,
		.modem_boot_on = msm_boot_on,
		.modem_boot_off = msm_boot_off,
	}
};

static struct resource mdmctl_res[] = {
	[0] = {
		.start = IRQ_PHONE_ACTIVE,
		.end = IRQ_PHONE_ACTIVE,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device modemctl = {
	.name = "modemctl",
	.id = -1,
	.num_resources = ARRAY_SIZE(mdmctl_res),
	.resource = mdmctl_res,
	.dev = {
		.platform_data = &mdmctl_data,
	},
};
#endif  /*CONFIG_SAMSUNG_PHONE_SVNET*/

