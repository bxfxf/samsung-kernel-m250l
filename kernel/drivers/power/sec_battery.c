/*
 *  sec_battery.c
 *  Samsung Mobile Battery Driver
 *
 *  Copyright (C) 2010 Samsung Electronics
 *
 *  <ms925.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/android_alarm.h>
#include <plat/adc.h>
#include <mach/sec_battery.h>

#define POLLING_INTERVAL	(40 * 1000)
#ifdef SEC_BATTERY_INDEPEDENT_VF_CHECK
#define VF_CHECK_INTERVAL	(10 * 1000)
#endif
#define FULL_CHARGING_TIME	(6 * 60 *  60 * HZ)	/* 6hr */
#define RECHARGING_TIME		(90 * 60 * HZ)	/* 1.5hr */
#define RESETTING_CHG_TIME	(10 * 60 * HZ)	/* 10Min */

#define RECHARGING_VOLTAGE	(4150 * 1000)	/* 4.15 V */

#define FG_T_SOC		0
#define FG_T_VCELL		1
#define FG_T_TEMPER		2
#define FG_T_PSOC		3
#define FG_T_VFOCV		4
#define FG_T_AVGVCELL	5

#define ADC_SAMPLING_CNT	7
#define ADC_CH_CHGCURRENT	1
#define ADC_TOTAL_COUNT		5

#define HIGH_BLOCK_TEMP			650
#define LOW_BLOCK_TEMP			(-30)

#define HIGH_RECOVER_TEMP		430
#define LOW_RECOVER_TEMP		0

#define CURRENT_OF_FULL_CHG			520
#define TEMP_BLOCK_COUNT			3
#ifdef SEC_BATTERY_INDEPEDENT_VF_CHECK
#define BAT_DET_COUNT				0
#else
#define BAT_DET_COUNT				1
#endif
#define FULL_CHG_COND_COUNT			3
#define FULL_CHARGE_COND_VOLTAGE    (4150 * 1000)	/* 4.15 V */

#define INIT_CHECK_COUNT	4

enum cable_type_t {
	CABLE_TYPE_NONE = 0,
	CABLE_TYPE_USB,
	CABLE_TYPE_AC,
	CABLE_TYPE_MISC,
};

enum batt_full_t {
	BATT_NOT_FULL = 0,
	BATT_FULL,
};

enum {
	BAT_NOT_DETECTED,
	BAT_DETECTED
};

static ssize_t sec_bat_show_property(struct device *dev,
				     struct device_attribute *attr, char *buf);

static ssize_t sec_bat_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count);

struct adc_sample {
	int average_adc;
	int adc_arr[ADC_TOTAL_COUNT];
	int index;
};

struct sec_bat_info {
	struct device *dev;

	char *fuel_gauge_name;
	char *charger_name;
	char *sub_charger_name;

	unsigned int adc_arr_size;
	struct sec_bat_adc_table_data *adc_table;
	unsigned int adc_channel;
	struct adc_sample temper_adc_sample;

	struct power_supply psy_bat;
	struct power_supply psy_usb;
	struct power_supply psy_ac;

	struct wake_lock vbus_wake_lock;
	struct wake_lock monitor_wake_lock;
	struct wake_lock cable_wake_lock;

	enum cable_type_t cable_type;
	enum batt_full_t batt_full_status;

	unsigned int batt_temp;	/* Battery Temperature (C) */
	int batt_temp_high_cnt;
	int batt_temp_low_cnt;
	int batt_temp_recover_cnt;
	unsigned int batt_health;
	unsigned int batt_vcell;
	unsigned int batt_vfocv;
	unsigned int batt_soc;
	unsigned int batt_raw_soc;
	unsigned int polling_interval;
	int charging_status;
	int charging_int_full_count;
	int charging_adc_full_count;

	unsigned int batt_temp_adc;
	unsigned int batt_current_adc;
	struct s3c_adc_client *padc;

	struct workqueue_struct *monitor_wqueue;
	struct work_struct monitor_work;
	struct work_struct cable_work;
	struct delayed_work polling_work;

	unsigned long charging_start_time;
	unsigned long charging_passed_time;
	unsigned long charging_next_time;
	unsigned int recharging_status;
	unsigned int batt_lpm_state;

	struct mutex adclock;

	unsigned int (*get_lpcharging_state) (void);
	int present;
	int present_count;
	bool use_sub_charger;

	int test_value;
	int initial_check_count;
	struct proc_dir_entry *entry;

#ifdef SEC_BATTERY_INDEPEDENT_VF_CHECK
	unsigned int vf_check_interval;
	struct delayed_work vf_check_work;
#endif
};

static char *supply_list[] = {
	"battery",
};

static enum power_supply_property sec_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static enum power_supply_property sec_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int calculate_average_adc(struct sec_bat_info *info,
				 struct adc_sample *sample, int adc)
{
	int i, total_adc = 0;
	int average_adc = sample->average_adc;
	int index = sample->index;

	if (adc < 0 || adc == 0) {
		dev_err(info->dev, "%s: invalid adc : %d\n", __func__, adc);
		return 0;
	}

	if (!average_adc) {
		average_adc = adc;
		for (i = 0; i < ADC_TOTAL_COUNT; i++)
			sample->adc_arr[i] = adc;
	} else {
		sample->index = ++index >= ADC_TOTAL_COUNT ? 0 : index;
		sample->adc_arr[index] = adc;
		for (i = 0; i < ADC_TOTAL_COUNT; i++)
			total_adc += sample->adc_arr[i];

		average_adc = total_adc / ADC_TOTAL_COUNT;
	}

	sample->average_adc = average_adc;
	dev_dbg(info->dev, "%s: i(%d) adc=%d, avg_adc=%d\n", __func__,
		sample->index, adc, average_adc);

	return average_adc;
}

static int sec_bat_get_fuelgauge_data(struct sec_bat_info *info, int type)
{
	struct power_supply *psy
	    = power_supply_get_by_name(info->fuel_gauge_name);
	union power_supply_propval value;

	if (!psy) {
		dev_err(info->dev, "%s: fail to get fuel gauge ps\n", __func__);
		return -ENODEV;
	}

	switch (type) {
	case FG_T_VCELL:
		value.intval = 0;	/*vcell */
		psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &value);
		break;
	case FG_T_VFOCV:
		value.intval = 1;	/*vfocv */
		psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &value);
		break;
	case FG_T_AVGVCELL:
		psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_AVG, &value);
		break;
	case FG_T_SOC:
		value.intval = 0;	/*normal soc */
		psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &value);
		break;
	case FG_T_PSOC:
		value.intval = 1;	/*raw soc */
		psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &value);
		break;
	case FG_T_TEMPER:
		psy->get_property(psy, POWER_SUPPLY_PROP_TEMP, &value);
		break;
	default:
		return -ENODEV;
	}

	return value.intval;
}

static int sec_bat_get_property(struct power_supply *ps,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct sec_bat_info *info = container_of(ps, struct sec_bat_info,
						 psy_bat);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = info->charging_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = info->batt_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = info->batt_temp;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		/* battery is always online */
		/* val->intval = 1; */
		val->intval = info->cable_type;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = info->batt_vcell;
		if (val->intval == -1)
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (info->charging_status == POWER_SUPPLY_STATUS_FULL) {
			val->intval = 100;
			break;
		}
		val->intval = info->batt_soc;
		if (val->intval == -1)
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sec_bat_handle_sub_charger_topoff(struct sec_bat_info *info)
{
	struct power_supply *psy_sub =
	    power_supply_get_by_name(info->sub_charger_name);
	union power_supply_propval value;

	info->charging_status = POWER_SUPPLY_STATUS_FULL;
	info->batt_full_status = BATT_FULL;
	info->recharging_status = false;
	/* disable charging */
	value.intval = POWER_SUPPLY_STATUS_DISCHARGING;
	return psy_sub->set_property(psy_sub, POWER_SUPPLY_PROP_STATUS, &value);
}

static int sec_bat_set_property(struct power_supply *ps,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct sec_bat_info *info = container_of(ps, struct sec_bat_info,
						 psy_bat);
	struct power_supply *psy = power_supply_get_by_name(info->charger_name);
	union power_supply_propval value;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		dev_info(info->dev, "%s: topoff intr\n", __func__);
		if (val->intval != POWER_SUPPLY_STATUS_FULL)
			return -EINVAL;

		if (info->use_sub_charger) {
			if (info->cable_type == CABLE_TYPE_USB ||
					info->cable_type == CABLE_TYPE_MISC)
				sec_bat_handle_sub_charger_topoff(info);
			break;
		}

		if (info->batt_full_status == BATT_NOT_FULL) {
			info->recharging_status = false;
			info->batt_full_status = BATT_FULL;
			info->charging_status = POWER_SUPPLY_STATUS_FULL;
			/* disable charging */
			value.intval = POWER_SUPPLY_STATUS_DISCHARGING;
			psy->set_property(psy, POWER_SUPPLY_PROP_STATUS,
					  &value);
		}
#if 0				/* for reference */
		if (info->batt_full_status == BATT_NOT_FULL) {
			info->batt_full_status = BATT_1ST_FULL;
			info->charging_status = POWER_SUPPLY_STATUS_FULL;
			/* TODO: set topoff current 60mA */
			value.intval = 120;
			psy->set_property(psy, POWER_SUPPLY_PROP_CHARGE_FULL,
					  &value);
		} else {
			info->batt_full_status = BATT_2ND_FULL;
			info->recharging_status = false;
			/* disable charging */
			value.intval = POWER_SUPPLY_STATUS_DISCHARGING;
			psy->set_property(psy, POWER_SUPPLY_PROP_STATUS,
					  &value);
		}
#endif
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		/* TODO: lowbatt interrupt: called by fuel gauge */
		dev_info(info->dev, "%s: lowbatt intr\n", __func__);
		if (val->intval != POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL)
			return -EINVAL;
		wake_lock(&info->monitor_wake_lock);
		queue_work(info->monitor_wqueue, &info->monitor_work);

		break;
	case POWER_SUPPLY_PROP_ONLINE:
		/* cable is attached or detached. called by USB switch(MUIC) */
		dev_info(info->dev, "%s: cable was changed(%d)\n", __func__,
			 val->intval);
		switch (val->intval) {
		case POWER_SUPPLY_TYPE_BATTERY:
			info->cable_type = CABLE_TYPE_NONE;
			break;
		case POWER_SUPPLY_TYPE_MAINS:
			info->cable_type = CABLE_TYPE_AC;
			break;
		case POWER_SUPPLY_TYPE_USB:
			info->cable_type = CABLE_TYPE_USB;
			break;
		case POWER_SUPPLY_TYPE_MISC:
			info->cable_type = CABLE_TYPE_MISC;
			break;
		default:
			return -EINVAL;
		}
		wake_lock(&info->cable_wake_lock);
		queue_work(info->monitor_wqueue, &info->cable_work);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sec_usb_get_property(struct power_supply *ps,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct sec_bat_info *info = container_of(ps, struct sec_bat_info,
						 psy_usb);

	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	/* Set enable=1 only if the USB charger is connected */
	val->intval = (info->cable_type == CABLE_TYPE_USB);

	return 0;
}

static int sec_ac_get_property(struct power_supply *ps,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct sec_bat_info *info = container_of(ps, struct sec_bat_info,
						 psy_ac);

	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	/* Set enable=1 only if the AC charger is connected */
	val->intval = (info->cable_type == CABLE_TYPE_AC) ||
			(info->cable_type == CABLE_TYPE_MISC);

	return 0;
}

static int sec_bat_get_adc_data(struct sec_bat_info *info, int adc_ch)
{
	int adc_data;
	int adc_max = 0;
	int adc_min = 0;
	int adc_total = 0;
	int i;
	int err_value;

	for (i = 0; i < ADC_SAMPLING_CNT; i++) {
		adc_data = s3c_adc_read(info->padc, adc_ch);

		if (adc_data < 0) {
			dev_err(info->dev, "%s : err(%d) returned, skip read\n",
				__func__, adc_data);
			err_value = adc_data;
			goto err;
		}

		if (i != 0) {
			if (adc_data > adc_max)
				adc_max = adc_data;
			else if (adc_data < adc_min)
				adc_min = adc_data;
		} else {
			adc_max = adc_data;
			adc_min = adc_data;
		}
		adc_total += adc_data;
	}

	return (adc_total - adc_max - adc_min) / (ADC_SAMPLING_CNT - 2);
err:
	return err_value;
}

static inline int s3c_read_temper_adc(struct sec_bat_info *info)
{
	int adc;

	mutex_lock(&info->adclock);
	adc = sec_bat_get_adc_data(info, info->adc_channel);
	mutex_unlock(&info->adclock);
	if (adc <= 0)
		adc = info->batt_temp_adc;
    info->batt_temp_adc = adc;

	return adc;
}

static int sec_bat_check_temper(struct sec_bat_info *info)
{
	struct power_supply *psy
	    = power_supply_get_by_name(info->fuel_gauge_name);
	union power_supply_propval value;
	int ret;

	int temp;
	int temp_adc = s3c_read_temper_adc(info);
	int health = info->batt_health;
	int low = 0;
	int high = 0;
	int mid = 0;

	calculate_average_adc(info, &info->temper_adc_sample, temp_adc);

	if (!info->adc_table || !info->adc_arr_size) {
		/* using fake temp */
		temp = 300;
		info->batt_temp = temp;
		return temp;
	}
	high = info->adc_arr_size - 1;

	while (low <= high) {
		mid = (low + high) / 2;
		if (info->adc_table[mid].adc > temp_adc)
			high = mid - 1;
		else if (info->adc_table[mid].adc < temp_adc)
			low = mid + 1;
		else
			break;
	}
	temp = info->adc_table[mid].temperature;

	info->batt_temp = temp;

	if (temp >= HIGH_BLOCK_TEMP) {
		if (health != POWER_SUPPLY_HEALTH_OVERHEAT &&
		    health != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
			if (info->batt_temp_high_cnt < TEMP_BLOCK_COUNT)
				info->batt_temp_high_cnt++;
			dev_info(info->dev, "%s: high count = %d\n",
				__func__, info->batt_temp_high_cnt);
	} else if (temp <= HIGH_RECOVER_TEMP && temp >= LOW_RECOVER_TEMP) {
		if (health == POWER_SUPPLY_HEALTH_OVERHEAT ||
		    health == POWER_SUPPLY_HEALTH_COLD)
			if (info->batt_temp_recover_cnt < TEMP_BLOCK_COUNT)
				info->batt_temp_recover_cnt++;
			dev_info(info->dev, "%s: recovery count = %d\n",
				__func__, info->batt_temp_recover_cnt);
	} else if (temp <= LOW_BLOCK_TEMP) {
		if (health != POWER_SUPPLY_HEALTH_COLD &&
		    health != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
			if (info->batt_temp_low_cnt < TEMP_BLOCK_COUNT)
				info->batt_temp_low_cnt++;
			dev_info(info->dev, "%s: low count = %d\n",
				__func__, info->batt_temp_low_cnt);
	} else {
		info->batt_temp_high_cnt = 0;
		info->batt_temp_low_cnt = 0;
		info->batt_temp_recover_cnt = 0;
	}

	if (info->batt_temp_high_cnt >= TEMP_BLOCK_COUNT)
		info->batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (info->batt_temp_low_cnt >= TEMP_BLOCK_COUNT)
		info->batt_health = POWER_SUPPLY_HEALTH_COLD;
	else if (info->batt_temp_recover_cnt >= TEMP_BLOCK_COUNT)
		info->batt_health = POWER_SUPPLY_HEALTH_GOOD;

	/* Set temperature to fuel gauge */
	if (info->fuel_gauge_name) {
		value.intval = info->batt_temp / 10;
		ret = psy->set_property(psy, POWER_SUPPLY_PROP_TEMP, &value);
		if (ret) {
			dev_err(info->dev, "%s: fail to set temperature(%d)\n",
				__func__, ret);
		}
	}

	dev_info(info->dev, "%s: temp=%d, adc=%d\n", __func__, temp, temp_adc);

	return temp;
}

static void check_chgcurrent(struct sec_bat_info *info)
{
	unsigned long chg_current_adc = 0;

	mutex_lock(&info->adclock);
	chg_current_adc = sec_bat_get_adc_data(info, ADC_CH_CHGCURRENT);
	mutex_unlock(&info->adclock);
	if (chg_current_adc < 0)
		chg_current_adc = info->batt_current_adc;
	info->batt_current_adc = chg_current_adc;

	dev_dbg(info->dev,
		"[battery] charging current = %d\n", info->batt_current_adc);
}

static void sec_check_chgcurrent(struct sec_bat_info *info)
{
	switch (info->charging_status) {
	case POWER_SUPPLY_STATUS_FULL:
		if (info->recharging_status == false)
			break;
	case POWER_SUPPLY_STATUS_CHARGING:
		if (info->batt_vcell >= FULL_CHARGE_COND_VOLTAGE) {
			check_chgcurrent(info);

			if (info->batt_current_adc <= CURRENT_OF_FULL_CHG) {
				/*TA full charged */
				info->charging_adc_full_count++;
				if (info->charging_adc_full_count >=
				    FULL_CHG_COND_COUNT) {
					info->charging_adc_full_count = 0;
					sec_bat_handle_sub_charger_topoff(info);
				}
				dev_info(info->dev, "%s : ADC full cnt = %d\n",
					 __func__,
					 info->charging_adc_full_count);
			} else
				info->charging_adc_full_count = 0;
		}
		break;
	default:
		info->charging_adc_full_count = 0;
		info->batt_current_adc = 0;
		break;
	}
}

static void sec_bat_update_info(struct sec_bat_info *info)
{
	info->batt_raw_soc = sec_bat_get_fuelgauge_data(info, FG_T_PSOC);
	info->batt_soc = sec_bat_get_fuelgauge_data(info, FG_T_SOC);
	info->batt_vcell = sec_bat_get_fuelgauge_data(info, FG_T_VCELL);
	info->batt_vfocv = sec_bat_get_fuelgauge_data(info, FG_T_VFOCV);
}

static int sec_bat_enable_charging_main(struct sec_bat_info *info, bool enable)
{
	struct power_supply *psy = power_supply_get_by_name(info->charger_name);
	union power_supply_propval val_type, val_chg_current, val_topoff;
	int ret;

	if (!psy) {
		dev_err(info->dev, "%s: fail to get charger ps\n", __func__);
		return -ENODEV;
	}

	info->batt_full_status = BATT_NOT_FULL;

	if (enable) {		/* Enable charging */
		switch (info->cable_type) {
		case CABLE_TYPE_USB:
			val_type.intval = POWER_SUPPLY_STATUS_CHARGING;
			val_chg_current.intval = 450;	/* mA */
			break;
		case CABLE_TYPE_AC:
			val_type.intval = POWER_SUPPLY_STATUS_CHARGING;
			val_chg_current.intval = 650;	/* mA */
			break;
		case CABLE_TYPE_MISC:
			val_type.intval = POWER_SUPPLY_STATUS_CHARGING;
			val_chg_current.intval = 450;	/* mA */
			break;
		default:
			dev_err(info->dev, "%s: Invalid func use\n", __func__);
			return -EINVAL;
		}

		/* Set charging current */
		ret = psy->set_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW,
					&val_chg_current);
		if (ret) {
			dev_err(info->dev, "%s: fail to set charging cur(%d)\n",
				__func__, ret);
			return ret;
		}

		/* Set topoff current */
		/* mA: TODO: should change 200->160 */
		val_topoff.intval = 200;
		ret = psy->set_property(psy, POWER_SUPPLY_PROP_CHARGE_FULL,
					&val_topoff);
		if (ret) {
			dev_err(info->dev, "%s: fail to set topoff cur(%d)\n",
				__func__, ret);
			return ret;
		}

		/*Reset charging start time only in initial charging start */
		if (info->charging_start_time == 0) {
			info->charging_start_time = jiffies;
			info->charging_next_time = RESETTING_CHG_TIME;
		}
	} else {			/* Disable charging */
		val_type.intval = POWER_SUPPLY_STATUS_DISCHARGING;

		info->charging_start_time = 0;
		info->charging_passed_time = 0;
		info->charging_next_time = 0;
	}

	ret = psy->set_property(psy, POWER_SUPPLY_PROP_STATUS, &val_type);
	if (ret) {
		dev_err(info->dev, "%s: fail to set charging status(%d)\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static int sec_bat_enable_charging_sub(struct sec_bat_info *info, bool enable)
{
	struct power_supply *psy_main =
	    power_supply_get_by_name(info->charger_name);
	struct power_supply *psy_sub =
	    power_supply_get_by_name(info->sub_charger_name);
	union power_supply_propval val_type, val_chg_current;
	int ret;

	if (!psy_main || !psy_sub) {
		dev_err(info->dev, "%s: fail to get charger ps\n", __func__);
		return -ENODEV;
	}

	info->batt_full_status = BATT_NOT_FULL;

	if (enable) {		/* Enable charging */
		val_type.intval = POWER_SUPPLY_STATUS_DISCHARGING;
		ret = psy_main->set_property(psy_main, POWER_SUPPLY_PROP_STATUS,
					     &val_type);
		if (ret) {
			dev_err(info->dev, "%s: fail to set charging"
				" status-main(%d)\n", __func__, ret);
			return ret;
		}

		switch (info->cable_type) {
		case CABLE_TYPE_USB:
			val_type.intval = POWER_SUPPLY_STATUS_CHARGING;
			val_chg_current.intval = 450;	/* mA */
			break;
		case CABLE_TYPE_AC:
			val_type.intval = POWER_SUPPLY_STATUS_CHARGING;
			val_chg_current.intval = 650;	/* mA */
			break;
		case CABLE_TYPE_MISC:
			val_type.intval = POWER_SUPPLY_STATUS_CHARGING;
			val_chg_current.intval = 450;	/* mA */
			break;
		default:
			dev_err(info->dev, "%s: Invalid func use\n", __func__);
			return -EINVAL;
		}

		/* Set charging current */
		ret = psy_sub->set_property(psy_sub,
					    POWER_SUPPLY_PROP_CURRENT_NOW,
					    &val_chg_current);
		if (ret) {
			dev_err(info->dev, "%s: fail to set charging cur(%d)\n",
				__func__, ret);
			return ret;
		}

		/*Reset charging start time only in initial charging start */
		if (info->charging_start_time == 0) {
			info->charging_start_time = jiffies;
			info->charging_next_time = RESETTING_CHG_TIME;
		}
	} else {		/* Disable charging */
		val_type.intval = POWER_SUPPLY_STATUS_DISCHARGING;
		ret = psy_main->set_property(psy_main, POWER_SUPPLY_PROP_STATUS,
					     &val_type);
		if (ret) {
			dev_err(info->dev, "%s: fail to set charging"
				" status-main(%d)\n", __func__, ret);
			return ret;
		}

		info->charging_start_time = 0;
		info->charging_passed_time = 0;
		info->charging_next_time = 0;
	}

	ret = psy_sub->set_property(psy_sub, POWER_SUPPLY_PROP_STATUS,
				    &val_type);
	if (ret) {
		dev_err(info->dev, "%s: fail to set charging status(%d)\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static int sec_bat_enable_charging(struct sec_bat_info *info, bool enable)
{
	if (enable && (info->batt_health != POWER_SUPPLY_HEALTH_GOOD)) {
		info->charging_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		dev_info(info->dev, "%s: Battery is NOT good!!!\n", __func__);
		return -EPERM;
	}

	if (info->use_sub_charger)
		return sec_bat_enable_charging_sub(info, enable);
	else
		return sec_bat_enable_charging_main(info, enable);
}

static void sec_bat_cable_work(struct work_struct *work)
{
	struct sec_bat_info *info = container_of(work, struct sec_bat_info,
						 cable_work);

	switch (info->cable_type) {
	case CABLE_TYPE_NONE:
		if (!sec_bat_enable_charging(info, false)) {
			info->batt_full_status = BATT_NOT_FULL;
			info->recharging_status = false;
			info->charging_status = POWER_SUPPLY_STATUS_DISCHARGING;
			info->batt_health = POWER_SUPPLY_HEALTH_GOOD;
			info->batt_temp_high_cnt = 0;
			info->batt_temp_low_cnt = 0;
			info->batt_temp_recover_cnt = 0;
			wake_lock_timeout(&info->vbus_wake_lock, HZ * 5);
		}
		break;
	case CABLE_TYPE_USB:
	case CABLE_TYPE_AC:
	case CABLE_TYPE_MISC:
		if (!sec_bat_enable_charging(info, true)) {
		info->charging_status = POWER_SUPPLY_STATUS_CHARGING;
		wake_lock(&info->vbus_wake_lock);
		}
		break;
	default:
		dev_err(info->dev, "%s: Invalid cable type\n", __func__);
		break;;
	}

	power_supply_changed(&info->psy_ac);
	power_supply_changed(&info->psy_usb);

	wake_unlock(&info->cable_wake_lock);
}

static bool sec_bat_charging_time_management(struct sec_bat_info *info)
{
	if (info->charging_start_time == 0) {
		dev_info(info->dev, "%s: charging_start_time has never\
			 been used since initializing\n", __func__);
		return false;
	}

	if (jiffies >= info->charging_start_time)
		info->charging_passed_time =
			jiffies - info->charging_start_time;
	else
		info->charging_passed_time =
			0xFFFFFFFF - info->charging_start_time + jiffies;

	switch (info->charging_status) {
	case POWER_SUPPLY_STATUS_FULL:
		if (time_after(info->charging_passed_time,
			(unsigned long)RECHARGING_TIME) &&
		    info->recharging_status == true) {
			if (!sec_bat_enable_charging(info, false)) {
			info->recharging_status = false;
				dev_info(info->dev,
					 "%s: Recharging timer expired\n",
				 __func__);
			return true;
		}
		}
		break;
	case POWER_SUPPLY_STATUS_CHARGING:
		if (time_after(info->charging_passed_time,
			       (unsigned long)FULL_CHARGING_TIME)) {
			if (!sec_bat_enable_charging(info, false)) {
				info->charging_status =
				    POWER_SUPPLY_STATUS_FULL;

				dev_info(info->dev,
					 "%s: Charging timer expired\n",
				 __func__);
			return true;
			}
		} else if (time_after(info->charging_passed_time,
					info->charging_next_time)) {
			/*reset current in charging status */
			if (!sec_bat_enable_charging(info, true)) {
			info->charging_next_time =
					info->charging_passed_time + RESETTING_CHG_TIME;

				dev_info(info->dev,
					 "%s: Reset charging current\n",
					 __func__);
			}
		}
		break;
	default:
		dev_info(info->dev, "%s: Undefine Battery Status\n", __func__);
		return false;
	}

	dev_info(info->dev, "Time past : %u secs\n",
		jiffies_to_msecs(info->charging_passed_time) / 1000);

	return false;
}

static void sec_bat_check_vf(struct sec_bat_info *info)
{
	struct power_supply *psy = power_supply_get_by_name(info->charger_name);
	union power_supply_propval value;
	int ret;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_PRESENT, &value);

	if (ret < 0) {
		dev_err(info->dev, "%s: fail to get status(%d)\n",
			__func__, ret);
		return;
	}

	if (value.intval == BAT_NOT_DETECTED) {
		if (info->present_count < BAT_DET_COUNT)
			info->present_count++;
		else {
			info->present = BAT_NOT_DETECTED;
			if (info->batt_health == POWER_SUPPLY_HEALTH_GOOD)
				info->batt_health =
				    POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		}
	} else {
		info->present = BAT_DETECTED;
		if ((info->batt_health == POWER_SUPPLY_HEALTH_UNSPEC_FAILURE) ||
		    (info->batt_health == POWER_SUPPLY_HEALTH_UNKNOWN))
			info->batt_health = POWER_SUPPLY_HEALTH_GOOD;
		info->present_count = 0;
	}

	return;
}

static bool sec_bat_check_ing_level_trigger(struct sec_bat_info *info)
{
	struct power_supply *psy_sub =
	    power_supply_get_by_name(info->sub_charger_name);
	union power_supply_propval value;
	int ret;

	ret = psy_sub->get_property(psy_sub, POWER_SUPPLY_PROP_STATUS, &value);

	if (ret < 0) {
		dev_err(info->dev, "%s: fail to get status(%d)\n",
			__func__, ret);
		return false;
	}

	switch (info->charging_status) {
	case POWER_SUPPLY_STATUS_FULL:
		if (info->recharging_status == false)
			break;
	case POWER_SUPPLY_STATUS_CHARGING:
		switch (value.intval) {
		case POWER_SUPPLY_STATUS_DISCHARGING:
			if (info->cable_type != CABLE_TYPE_NONE)
				info->batt_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			break;
		case POWER_SUPPLY_STATUS_NOT_CHARGING:
			if (info->cable_type == CABLE_TYPE_USB ||
					info->cable_type == CABLE_TYPE_MISC) {
				if (info->batt_vcell >=
				    FULL_CHARGE_COND_VOLTAGE) {
					/*USB full charged */
					info->charging_int_full_count++;
					if (info->charging_int_full_count >=
						FULL_CHG_COND_COUNT) {
						info->charging_int_full_count =
						    0;
						sec_bat_handle_sub_charger_topoff
						    (info);
						return true;
					}
					dev_info(info->dev,
						"%s : full interrupt cnt = %d\n",
						__func__,
						info->charging_int_full_count);
				} else {
					info->charging_int_full_count = 0;
					/*reactivate charging in next monitor work
					    for abnormal full-charged status */
					info->charging_next_time =
						info->charging_passed_time + HZ;
				}
			}
			break;
		}
		break;
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		if (info->batt_health == POWER_SUPPLY_HEALTH_OVERVOLTAGE &&
		    value.intval == POWER_SUPPLY_STATUS_NOT_CHARGING)
			info->batt_health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	default:
		info->charging_int_full_count = 0;
		break;
	}
	return false;
}

static void sec_bat_monitor_work(struct work_struct *work)
{
	struct sec_bat_info *info = container_of(work, struct sec_bat_info,
						 monitor_work);

	sec_bat_check_temper(info);
#ifndef SEC_BATTERY_INDEPEDENT_VF_CHECK
	sec_bat_check_vf(info);
#endif
	sec_bat_update_info(info);
	if (sec_bat_charging_time_management(info))
		goto full_charged;

	if (info->use_sub_charger) {
		if (sec_bat_check_ing_level_trigger(info))
			goto full_charged;
	}

	if (info->cable_type == CABLE_TYPE_AC)
		sec_check_chgcurrent(info);

	if (info->cable_type == CABLE_TYPE_NONE &&
	    info->charging_status == POWER_SUPPLY_STATUS_FULL) {
		dev_err(info->dev, "invalid full state\n");
		info->charging_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	if (info->test_value) {
		switch (info->test_value) {
		case 1: /*full status */
			info->charging_status = POWER_SUPPLY_STATUS_FULL;
			break;
		case 2: /*low temperature */
			info->batt_health = POWER_SUPPLY_HEALTH_COLD;
			break;
		case 3: /*high temperature */
			info->batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
			break;
		case 4: /*over voltage */
			info->batt_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			break;
		case 5: /*abnormal battery */
			info->batt_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			break;
		}
	}

	switch (info->charging_status) {
	case POWER_SUPPLY_STATUS_FULL:
		if (info->batt_vcell < RECHARGING_VOLTAGE &&
		    info->recharging_status == false) {
			if (!sec_bat_enable_charging(info, true)) {
			info->recharging_status = true;

			dev_info(info->dev,
					 "%s: Start Recharging, Vcell = %d\n",
					 __func__, info->batt_vcell);
			}
		}
		break;
	case POWER_SUPPLY_STATUS_CHARGING:
		switch (info->batt_health) {
		case POWER_SUPPLY_HEALTH_OVERHEAT:
		case POWER_SUPPLY_HEALTH_COLD:
		case POWER_SUPPLY_HEALTH_OVERVOLTAGE:
		case POWER_SUPPLY_HEALTH_DEAD:
		case POWER_SUPPLY_HEALTH_UNSPEC_FAILURE:
			if (!sec_bat_enable_charging(info, false)) {
			info->charging_status =
			    POWER_SUPPLY_STATUS_NOT_CHARGING;
			info->recharging_status = false;

				dev_info(info->dev, "%s: Not charging\n",
					 __func__);
			}
			break;
		default:
			break;
		}
		break;
	case POWER_SUPPLY_STATUS_DISCHARGING:
		dev_dbg(info->dev, "%s: Discharging\n", __func__);
		break;
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		if (info->batt_health == POWER_SUPPLY_HEALTH_GOOD) {
			dev_info(info->dev, "%s: recover health state\n",
				 __func__);
			if (info->cable_type != CABLE_TYPE_NONE) {
				if (!sec_bat_enable_charging(info, true))
				info->charging_status
				    = POWER_SUPPLY_STATUS_CHARGING;
			} else
				info->charging_status
				    = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		break;
	default:
		dev_info(info->dev, "%s: Undefined Battery Status\n", __func__);
		return;
	}

full_charged:
	dev_info(info->dev,
		"soc(%d), vfocv(%d), vcell(%d), temp(%d), charging(%d), health(%d)\n",
		info->batt_soc, info->batt_vfocv,
		info->batt_vcell / 1000, info->batt_temp / 10,
		info->charging_status, info->batt_health);

	power_supply_changed(&info->psy_bat);

	wake_unlock(&info->monitor_wake_lock);

	return;
}

#ifdef SEC_BATTERY_INDEPEDENT_VF_CHECK
static void sec_bat_vf_check_work(struct work_struct *work)
{
	struct sec_bat_info *info;
	info = container_of(work, struct sec_bat_info, vf_check_work.work);

	sec_bat_check_vf(info);

	if (info->batt_health == POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
		power_supply_changed(&info->psy_bat);

	schedule_delayed_work(&info->vf_check_work,
			      msecs_to_jiffies(VF_CHECK_INTERVAL));
}
#endif

static void sec_bat_polling_work(struct work_struct *work)
{
	struct sec_bat_info *info;
	info = container_of(work, struct sec_bat_info, polling_work.work);

	wake_lock(&info->monitor_wake_lock);
	queue_work(info->monitor_wqueue, &info->monitor_work);

	if (info->initial_check_count) {
		schedule_delayed_work(&info->polling_work, HZ);
		info->initial_check_count--;
	} else
	schedule_delayed_work(&info->polling_work,
			      msecs_to_jiffies(info->polling_interval));
}

#define SEC_BATTERY_ATTR(_name)			\
{						\
	.attr = { .name = #_name,		\
		  .mode = 0664,			\
		  .owner = THIS_MODULE },	\
	.show = sec_bat_show_property,		\
	.store = sec_bat_store,			\
}

static struct device_attribute sec_battery_attrs[] = {
	SEC_BATTERY_ATTR(batt_vol),
	SEC_BATTERY_ATTR(batt_soc),
	SEC_BATTERY_ATTR(batt_vfocv),
	SEC_BATTERY_ATTR(batt_temp),
	SEC_BATTERY_ATTR(batt_temp_adc),
	SEC_BATTERY_ATTR(batt_temp_adc_avg),
	SEC_BATTERY_ATTR(charging_source),
	SEC_BATTERY_ATTR(batt_lp_charging),
	SEC_BATTERY_ATTR(video),
	SEC_BATTERY_ATTR(mp3),
	SEC_BATTERY_ATTR(batt_type),
	SEC_BATTERY_ATTR(batt_full_check),
	SEC_BATTERY_ATTR(batt_temp_check),
	SEC_BATTERY_ATTR(batt_temp_adc_spec),
	SEC_BATTERY_ATTR(batt_test_value),
	SEC_BATTERY_ATTR(batt_current_adc),
	SEC_BATTERY_ATTR(system_rev),
	SEC_BATTERY_ATTR(fg_psoc),
	SEC_BATTERY_ATTR(batt_lpm_state),
};

enum {
	BATT_VOL = 0,
	BATT_SOC,
	BATT_VFOCV,
	BATT_TEMP,
	BATT_TEMP_ADC,
	BATT_TEMP_ADC_AVG,
	CHARGING_SOURCE,
	BATT_LP_CHARGING,
	BATT_VIDEO,
	BATT_MP3,
	BATT_TYPE,
	BATT_FULL_CHECK,
	BATT_TEMP_CHECK,
	BATT_TEMP_ADC_SPEC,
	BATT_TEST_VALUE,
	BATT_CURRENT_ADC,
	BATT_SYSTEM_REV,
	BATT_FG_PSOC,
	BATT_LPM_STATE,
};

static ssize_t sec_bat_show_property(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct sec_bat_info *info = dev_get_drvdata(dev->parent);
	int i = 0, val;
	const ptrdiff_t off = attr - sec_battery_attrs;

	switch (off) {
	case BATT_VOL:
		val = sec_bat_get_fuelgauge_data(info, FG_T_AVGVCELL);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", val);
		break;
	case BATT_SOC:
		val = sec_bat_get_fuelgauge_data(info, FG_T_SOC);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", val);
		break;
	case BATT_VFOCV:
		val = sec_bat_get_fuelgauge_data(info, FG_T_VFOCV);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", val);
		break;
	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", info->batt_temp);
		break;
	case BATT_TEMP_ADC:
		val = s3c_read_temper_adc(info);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", val);
		break;
	case BATT_TEMP_ADC_AVG:
		val = info->temper_adc_sample.average_adc;
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", val);
		break;
	case CHARGING_SOURCE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       info->cable_type);
		break;
	case BATT_LP_CHARGING:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       info->get_lpcharging_state());
		break;
	case BATT_VIDEO:
		/* TODO */
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", 0);
		break;
	case BATT_MP3:
		/* TODO */
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", 0);
		break;
	case BATT_TYPE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n", "SDI_SDI");
		break;
	case BATT_FULL_CHECK:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       (info->charging_status ==
				POWER_SUPPLY_STATUS_FULL)
			? 1 : 0);
		break;
	case BATT_TEMP_CHECK:
		i += scnprintf(buf + i, PAGE_SIZE - i,
			"%d\n", info->batt_health);
		break;
	case BATT_TEMP_ADC_SPEC:
		i += scnprintf(buf + i, PAGE_SIZE - i,
			"(HIGH: %d - %d,   LOW: %d - %d)\n",
			       HIGH_BLOCK_TEMP / 10, HIGH_RECOVER_TEMP / 10,
			       LOW_BLOCK_TEMP / 10, LOW_RECOVER_TEMP / 10);
		break;
	case BATT_TEST_VALUE:
		i += scnprintf(buf + i, PAGE_SIZE - i,
			"0-normal, 1-full, 2-low, 3-high, 4-over, 5-cf (%d)\n",
			info->test_value);
		break;
	case BATT_CURRENT_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			info->batt_current_adc);
		break;
	case BATT_SYSTEM_REV:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", system_rev);
		break;
	case BATT_FG_PSOC:
		val = sec_bat_get_fuelgauge_data(info, FG_T_PSOC);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", val);
		break;
	case BATT_LPM_STATE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			info->batt_lpm_state);
	default:
		i = -EINVAL;
	}

	return i;
}

static ssize_t sec_bat_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int ret = 0, x = 0;
	const ptrdiff_t off = attr - sec_battery_attrs;
	struct sec_bat_info *info = dev_get_drvdata(dev->parent);

	switch (off) {
	case BATT_VIDEO:
		/* TODO */
		if (sscanf(buf, "%d\n", &x) == 1) {
			dev_info(info->dev, "%s: video(%d)\n", __func__, x);
			ret = count;
		}
		break;
	case BATT_MP3:
		/* TODO */
		if (sscanf(buf, "%d\n", &x) == 1) {
			dev_info(info->dev, "%s: mp3(%d)\n", __func__, x);
			ret = count;
		}
		break;
	case BATT_TEST_VALUE:
		if (sscanf(buf, "%d\n", &x) == 1) {
			info->test_value = x;
			printk("%s : test case : %d\n", __func__,
			       info->test_value);
			ret = count;
		}
		break;
	case BATT_LPM_STATE:
		if (sscanf(buf, "%d\n", &x) == 1) {
			info->batt_lpm_state = x;
			ret = count;
		}
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int sec_bat_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(sec_battery_attrs); i++) {
		rc = device_create_file(dev, &sec_battery_attrs[i]);
		if (rc)
			goto sec_attrs_failed;
	}
	goto succeed;

sec_attrs_failed:
	while (i--)
		device_remove_file(dev, &sec_battery_attrs[i]);
succeed:
	return rc;
}

static int sec_bat_is_charging(struct sec_bat_info *info)
{
	struct power_supply *psy = power_supply_get_by_name(info->charger_name);
	union power_supply_propval value;
	int ret;

	if (!psy) {
		dev_err(info->dev, "%s: fail to get charger ps\n", __func__);
		return -ENODEV;
	}

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_STATUS, &value);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to get status(%d)\n", __func__,
			ret);
		return ret;
	}

	return value.intval;
}

static int sec_bat_read_proc(char *buf, char **start,
			off_t offset, int count, int *eof, void *data)
{
	struct sec_bat_info *info = data;
	struct timespec cur_time;
	ktime_t ktime;
	int len = 0;

	ktime = alarm_get_elapsed_realtime();
	cur_time = ktime_to_timespec(ktime);

	len = sprintf(buf, "%lu, %u, %u, %u, %u, %u, %d, %d, %d, \
%u, %u, %u, %u, %u, %d, %lu\n",
		cur_time.tv_sec,
		info->batt_raw_soc,
		info->batt_soc,
		info->batt_vfocv,
		info->batt_vcell,
		info->batt_current_adc,
		info->batt_full_status,
		info->charging_int_full_count,
		info->charging_adc_full_count,
		info->recharging_status,
		info->batt_temp_adc,
		info->batt_health,
		info->charging_status,
		info->present,
		info->cable_type,
		info->charging_passed_time);

    return len;
}

static __devinit int sec_bat_probe(struct platform_device *pdev)
{
	struct sec_bat_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct sec_bat_info *info;
	struct power_supply *psy;
	union power_supply_propval value;
	int ret = 0;

	dev_info(&pdev->dev, "%s: SEC Battery Driver Loading\n", __func__);

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	platform_set_drvdata(pdev, info);

	info->dev = &pdev->dev;
	if (!pdata->fuel_gauge_name || !pdata->charger_name) {
		dev_err(info->dev, "%s: no fuel gauge or charger name\n",
			__func__);
		goto err_kfree;
	}
	info->fuel_gauge_name = pdata->fuel_gauge_name;
	info->charger_name = pdata->charger_name;
	if (system_rev >= HWREV_FOR_BATTERY) {
		dev_info(&pdev->dev, "%s: use sub-charger\n", __func__);
		info->adc_arr_size = pdata->adc_sub_arr_size;
		info->adc_table = pdata->adc_sub_table;
		info->adc_channel = pdata->adc_sub_channel;
	} else {
		dev_info(&pdev->dev, "%s: use main charger\n", __func__);
		info->adc_arr_size = pdata->adc_arr_size;
		info->adc_table = pdata->adc_table;
		info->adc_channel = pdata->adc_channel;
	}

	info->get_lpcharging_state = pdata->get_lpcharging_state;

	if (pdata->sub_charger_name) {
		info->sub_charger_name = pdata->sub_charger_name;
		if (system_rev >= HWREV_FOR_BATTERY)
			info->use_sub_charger = true;
	}

	info->psy_bat.name = "battery",
	    info->psy_bat.type = POWER_SUPPLY_TYPE_BATTERY,
	    info->psy_bat.properties = sec_battery_props,
	    info->psy_bat.num_properties = ARRAY_SIZE(sec_battery_props),
	    info->psy_bat.get_property = sec_bat_get_property,
	    info->psy_bat.set_property = sec_bat_set_property,
	    info->psy_usb.name = "usb",
	    info->psy_usb.type = POWER_SUPPLY_TYPE_USB,
	    info->psy_usb.supplied_to = supply_list,
	    info->psy_usb.num_supplicants = ARRAY_SIZE(supply_list),
	    info->psy_usb.properties = sec_power_props,
	    info->psy_usb.num_properties = ARRAY_SIZE(sec_power_props),
	    info->psy_usb.get_property = sec_usb_get_property,
	    info->psy_ac.name = "ac",
	    info->psy_ac.type = POWER_SUPPLY_TYPE_MAINS,
	    info->psy_ac.supplied_to = supply_list,
	    info->psy_ac.num_supplicants = ARRAY_SIZE(supply_list),
	    info->psy_ac.properties = sec_power_props,
	    info->psy_ac.num_properties = ARRAY_SIZE(sec_power_props),
	    info->psy_ac.get_property = sec_ac_get_property;

	wake_lock_init(&info->vbus_wake_lock, WAKE_LOCK_SUSPEND,
		       "vbus_present");
	wake_lock_init(&info->monitor_wake_lock, WAKE_LOCK_SUSPEND,
		       "sec-battery-monitor");
	wake_lock_init(&info->cable_wake_lock, WAKE_LOCK_SUSPEND,
		       "sec-battery-cable");

	psy = power_supply_get_by_name(info->charger_name);

	if (!psy) {
		dev_err(info->dev, "%s: fail to get charger\n", __func__);
		return -ENODEV;
	}

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_PRESENT, &value);

	if (ret < 0) {
		dev_err(info->dev, "%s: fail to get status(%d)\n",
			__func__, ret);
		return -ENODEV;
	}

	info->present = value.intval;
	
	if (info->present == BAT_NOT_DETECTED)
		info->batt_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	else
	info->batt_health = POWER_SUPPLY_HEALTH_GOOD;

	info->charging_status = sec_bat_is_charging(info);
	if (info->charging_status < 0)
		info->charging_status = POWER_SUPPLY_STATUS_DISCHARGING;

	info->present_count = 0;
	info->charging_int_full_count = 0;
	info->charging_adc_full_count = 0;
	info->batt_temp_high_cnt = 0;
	info->batt_temp_low_cnt = 0;
	info->batt_temp_recover_cnt = 0;

	info->initial_check_count = INIT_CHECK_COUNT;

	mutex_init(&info->adclock);

	info->padc = s3c_adc_register(pdev, NULL, NULL, 0);
	info->charging_start_time = 0;

	if (info->get_lpcharging_state) {
		if (info->get_lpcharging_state())
			info->polling_interval = POLLING_INTERVAL / 4;
		else
			info->polling_interval = POLLING_INTERVAL;
	}

	info->test_value = 0;

	/* init power supplier framework */
	ret = power_supply_register(&pdev->dev, &info->psy_bat);
	if (ret) {
		dev_err(info->dev, "%s: failed to register psy_bat\n",
			__func__);
		goto err_wake_lock;
	}

	ret = power_supply_register(&pdev->dev, &info->psy_usb);
	if (ret) {
		dev_err(info->dev, "%s: failed to register psy_usb\n",
			__func__);
		goto err_supply_unreg_bat;
	}

	ret = power_supply_register(&pdev->dev, &info->psy_ac);
	if (ret) {
		dev_err(info->dev, "%s: failed to register psy_ac\n", __func__);
		goto err_supply_unreg_usb;
	}

	/* create sec detail attributes */
	sec_bat_create_attrs(info->psy_bat.dev);

	info->entry = create_proc_entry("batt_info_proc", S_IRUGO, NULL);
	if (!info->entry)
		dev_err(info->dev, "%s: failed to create proc_entry\n",
			__func__);
    else {
		info->entry->read_proc = sec_bat_read_proc;
		info->entry->data = (struct sec_bat_info *)info;
    }

	info->monitor_wqueue =
	    create_freezeable_workqueue(dev_name(&pdev->dev));
	if (!info->monitor_wqueue) {
		dev_err(info->dev, "%s: fail to create workqueue\n", __func__);
		goto err_supply_unreg_ac;
	}

	INIT_WORK(&info->monitor_work, sec_bat_monitor_work);
	INIT_WORK(&info->cable_work, sec_bat_cable_work);

	INIT_DELAYED_WORK_DEFERRABLE(&info->polling_work,
		sec_bat_polling_work);
	schedule_delayed_work(&info->polling_work, 0);

#ifdef SEC_BATTERY_INDEPEDENT_VF_CHECK
	INIT_DELAYED_WORK_DEFERRABLE(&info->vf_check_work,
		sec_bat_vf_check_work);
	schedule_delayed_work(&info->vf_check_work, 0);
#endif

	return 0;

err_supply_unreg_ac:
	power_supply_unregister(&info->psy_ac);
err_supply_unreg_usb:
	power_supply_unregister(&info->psy_usb);
err_supply_unreg_bat:
	power_supply_unregister(&info->psy_bat);
err_wake_lock:
	wake_lock_destroy(&info->vbus_wake_lock);
	wake_lock_destroy(&info->monitor_wake_lock);
	wake_lock_destroy(&info->cable_wake_lock);
	mutex_destroy(&info->adclock);
err_kfree:
	kfree(info);

	return ret;
}

static int __devexit sec_bat_remove(struct platform_device *pdev)
{
	struct sec_bat_info *info = platform_get_drvdata(pdev);

	remove_proc_entry("batt_info_proc", NULL);

	flush_workqueue(info->monitor_wqueue);
	destroy_workqueue(info->monitor_wqueue);

	cancel_delayed_work(&info->polling_work);
#ifdef SEC_BATTERY_INDEPEDENT_VF_CHECK
	cancel_delayed_work(&info->vf_check_work);
#endif

	power_supply_unregister(&info->psy_bat);
	power_supply_unregister(&info->psy_usb);
	power_supply_unregister(&info->psy_ac);

	wake_lock_destroy(&info->vbus_wake_lock);
	wake_lock_destroy(&info->monitor_wake_lock);
	wake_lock_destroy(&info->cable_wake_lock);
	mutex_destroy(&info->adclock);

	s3c_adc_release(info->padc);

	kfree(info);

	return 0;
}

static int sec_bat_suspend(struct device *dev)
{
	struct sec_bat_info *info = dev_get_drvdata(dev);

	cancel_work_sync(&info->monitor_work);
	cancel_delayed_work(&info->polling_work);
#ifdef SEC_BATTERY_INDEPEDENT_VF_CHECK
	cancel_delayed_work(&info->vf_check_work);
#endif

	return 0;
}

static int sec_bat_resume(struct device *dev)
{
	struct sec_bat_info *info = dev_get_drvdata(dev);

	wake_lock(&info->monitor_wake_lock);
	queue_work(info->monitor_wqueue, &info->monitor_work);

	schedule_delayed_work(&info->polling_work,
			      msecs_to_jiffies(info->polling_interval));

#ifdef SEC_BATTERY_INDEPEDENT_VF_CHECK
	schedule_delayed_work(&info->vf_check_work,
			msecs_to_jiffies(VF_CHECK_INTERVAL));
#endif

	return 0;
}

static const struct dev_pm_ops sec_bat_pm_ops = {
	.suspend = sec_bat_suspend,
	.resume = sec_bat_resume,
};

static struct platform_driver sec_bat_driver = {
	.driver = {
		   .name = "sec-battery",
		   .owner = THIS_MODULE,
		   .pm = &sec_bat_pm_ops,
		   },
	.probe = sec_bat_probe,
	.remove = __devexit_p(sec_bat_remove),
};

static int __init sec_bat_init(void)
{
	return platform_driver_register(&sec_bat_driver);
}

static void __exit sec_bat_exit(void)
{
	platform_driver_unregister(&sec_bat_driver);
}

late_initcall(sec_bat_init);
module_exit(sec_bat_exit);

MODULE_DESCRIPTION("SEC battery driver");
MODULE_AUTHOR("<ms925.kim@samsung.com>");
MODULE_AUTHOR("<joshua.chang@samsung.com>");
MODULE_LICENSE("GPL");
