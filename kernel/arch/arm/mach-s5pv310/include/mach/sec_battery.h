/*
 * Copyright (C) 2010 Samsung Electronics, Inc.
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

#ifndef __MACH_SEC_BATTERY_H
#define __MACH_SEC_BATTERY_H __FILE__

#if defined(CONFIG_TARGET_LOCALE_KOR)||defined(CONFIG_TARGET_LOCALE_NTT)
#define HWREV_FOR_BATTERY	0x03 /*Modified for IGNIS-L by mseok.chae. If you wanna merge it please check this.*/
#else	/*U1 EUR OPEN */
#define HWREV_FOR_BATTERY	0x08
#endif

/*soc level for 3.6V */
#define SEC_BATTERY_SOC_3_6	7

#define SEC_BATTERY_INDEPEDENT_VF_CHECK

/**
 * struct sec_bat_adc_table_data - adc to temperature table for sec battery
 * driver
 * @adc: adc value
 * @temperature: temperature(C) * 10
 */
struct sec_bat_adc_table_data {
	int adc;
	int temperature;
};

/**
 * struct sec_bat_plaform_data - init data for sec batter driver
 * @fuel_gauge_name: power supply name of fuel gauge
 * @charger_name: power supply name of charger
 * @sub_charger_name: power supply name of sub-charger
 * @adc_table: array of adc to temperature data
 * @adc_arr_size: size of adc_table
 * @irq_topoff: IRQ number for top-off interrupt
 * @irq_lowbatt: IRQ number for low battery alert interrupt
 */
struct sec_bat_platform_data {
	char *fuel_gauge_name;
	char *charger_name;
	char *sub_charger_name;

	unsigned int adc_arr_size;
	struct sec_bat_adc_table_data *adc_table;
	unsigned int adc_channel;
	unsigned int adc_sub_arr_size;
	struct sec_bat_adc_table_data *adc_sub_table;
	unsigned int adc_sub_channel;
	
	unsigned int (*get_lpcharging_state) (void);
	void (*no_bat_cb) (void);
};

#endif /* __MACH_SEC_BATTERY_H */
