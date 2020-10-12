// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * corsair-clink.c - Linux driver for Corsair Link based PSUs
 * Copyright (C) 2020 Denis Biryukov  <denis.biruko@gmail.com>
 *
 * Based on corsair-cpro by 2020 Marius Zachmann
 *
 * This driver uses hid reports to communicate with the device to allow hidraw userspace drivers
 * still being used. The device does not use report ids. When using hidraw and this driver
 * simultaniously, reports could be switched.
 */

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/hid.h>
#include <linux/hwmon.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>


#define USB_VENDOR_ID_CORSAIR   0x1b1c

#define OUT_BUFFER_SIZE		64
#define IN_BUFFER_SIZE		64
#define LABEL_LENGTH		16
#define REQ_TIMEOUT		300

#define CMD_GET_ID  0x00

struct clink_device {
	struct hid_device *hdev;
	struct device *hwmon_dev;
	struct completion wait_input_report;
	struct mutex mutex; /* whenever buffer is used, lock before send_usb_cmd */
	u8 *buffer;
};

static const char channel_labels[4][LABEL_LENGTH] = { "Power supply", "+12V", "+5V", "+3.3V"};

/* converts response error in buffer to errno */
static int clink_get_errno(struct clink_device *clink)
{
	switch (clink->buffer[0]) {
	case 0x00: /* success */
		return 0;
	case 0x01: /* called invalid command */
		return -EOPNOTSUPP;
	case 0x10: /* called GET_VOLT / GET_TMP with invalid arguments */
		return -EINVAL;
	case 0x11: /* requested temps of disconnected sensors */
	case 0x12: /* requested pwm of not pwm controlled channels */
		return -ENODATA;
	default:
		hid_dbg(clink->hdev, "unknown device response error: %d", clink->buffer[0]);
		return -EIO;
	}
}

/* send command, check for error in response, response in clink->buffer */
static int send_usb_cmd(struct clink_device *clink, u8 command, u8 byte1, u8 byte2, u8 byte3)
{
	unsigned long t;
	int ret;

	memset(clink->buffer, 0x00, OUT_BUFFER_SIZE);
	clink->buffer[0] = command;
	clink->buffer[1] = byte1;
	clink->buffer[2] = byte2;
	clink->buffer[3] = byte3;

	reinit_completion(&clink->wait_input_report);

	ret = hid_hw_output_report(clink->hdev, clink->buffer, OUT_BUFFER_SIZE);
	if (ret < 0)
		return ret;

	t = wait_for_completion_timeout(&clink->wait_input_report, msecs_to_jiffies(REQ_TIMEOUT));
	if (!t)
		return -ETIMEDOUT;

	return clink_get_errno(clink);
}

static int clink_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *data, int size)
{
	struct clink_device *clink = hid_get_drvdata(hdev);

	/* only copy buffer when requested */
	if (completion_done(&clink->wait_input_report))
		return 0;

	memcpy(clink->buffer, data, min(IN_BUFFER_SIZE, size));
	complete(&clink->wait_input_report);

	return 0;
}

static int rmi_send_cmd(struct clink_device* clink, bool wait)
{
    int ret = 0;

    reinit_completion(&clink->wait_input_report);

    ret = hid_hw_output_report(clink->hdev, clink->buffer, OUT_BUFFER_SIZE);
    if (ret < 0)
        return ret;

    if (wait)
    {
        ret = wait_for_completion_timeout(&clink->wait_input_report, msecs_to_jiffies(REQ_TIMEOUT));
        if (!ret)
            return -ETIMEDOUT;
    }

    return ret;
}

static int rmi_temperature(
    struct clink_device* clink,
    uint8_t probe)
{
    int ret;
    uint16_t temp;

    clink->buffer[0] = 0x03;
    clink->buffer[1] = 0x8D + probe;
    clink->buffer[2] = 0x00;
    clink->buffer[3] = 0x00;

    ret = rmi_send_cmd(clink, true);
    if (ret < 0)
        return ret;

    temp = ( clink->buffer[2] << 8 ) + clink->buffer[3];

    return temp;
}

int pow2i(int exp)
{
	return (1<<exp);
}

int get_int_from_uint16_double(uint16_t data)
{
	uint32_t fraction = data & 2047;
	int32_t exponent = data >> 11;
	uint32_t result;
	
	if ( fraction > 1023 )
		fraction = -( 2048 - fraction );
	
	if ( ( fraction & 1 ) == 1 )
		fraction++;
	
	if ( exponent > 15 )
		exponent = -( 32 - exponent );
		
	result = fraction * 1000;
	if (exponent > 0)
	{
		result *= pow2i(exponent);
	}
	else
	{
		result /= pow2i(-exponent);
	}

	return result;
	
}

static int rmi_voltage(
    struct clink_device* clink,
    uint8_t probe)
{
    int ret;

    if (probe == 0) //Power suuply
    {
        clink->buffer[0] = 0x03;
        clink->buffer[1] = 0x88;
        clink->buffer[2] = 0x00;
        clink->buffer[3] = 0x00;
    }
    else
    {

        clink->buffer[0] = 0x02;
        clink->buffer[1] = 0x00;
        clink->buffer[2] = probe - 1;

        rmi_send_cmd(clink, true);

        clink->buffer[0] = 0x03;
        clink->buffer[1] = 0x8B;
        clink->buffer[2] = 0x00;
        clink->buffer[3] = 0x00;
    }


    ret = rmi_send_cmd(clink, true);
    if (ret < 0)
        return ret;

	uint16_t temp = ( clink->buffer[3] << 8 ) | clink->buffer[2];

	return get_int_from_uint16_double(temp);
}

static int rmi_power(
    struct clink_device* clink,
    uint8_t probe)
{
    int ret;

    if (probe == 0) //Power suuply
    {
        clink->buffer[0] = 0x03;
        clink->buffer[1] = 0xEE;
        clink->buffer[2] = 0x00;
        clink->buffer[3] = 0x00;
    }
    else
    {

        clink->buffer[0] = 0x02;
        clink->buffer[1] = 0x00;
        clink->buffer[2] = probe - 1;

        rmi_send_cmd(clink, true);

        clink->buffer[0] = 0x03;
        clink->buffer[1] = 0x96;
        clink->buffer[2] = 0x00;
        clink->buffer[3] = 0x00;
    }


    ret = rmi_send_cmd(clink, true);
    if (ret < 0)
        return ret;

    uint16_t temp = ( clink->buffer[3] << 8 ) | clink->buffer[2];

	return get_int_from_uint16_double(temp) * 1000;
}

static int rmi_current(
	struct clink_device* clink,
	uint8_t probe)
{
	int ret;
	
	clink->buffer[0] = 0x02;
	clink->buffer[1] = 0x00;
	clink->buffer[2] = probe;
	
	rmi_send_cmd(clink, true);
	
	clink->buffer[0] = 0x03;
	clink->buffer[1] = 0x8C;
	clink->buffer[2] = 0x00;
	clink->buffer[3] = 0x00;
	
	ret = rmi_send_cmd(clink, true);
	if (ret < 0)
		return ret;
	
	uint16_t temp = ( clink->buffer[3] << 8 ) | clink->buffer[2];
	
	return get_int_from_uint16_double(temp);
}

static int rmi_fan(
	struct clink_device* clink)
{
	int ret;
	
	clink->buffer[0] = 0x03;
	clink->buffer[1] = 0x90;
	clink->buffer[2] = 0x00;
	clink->buffer[3] = 0x00;
	
	ret = rmi_send_cmd(clink, true);
	if (ret < 0)
		return ret;
	
	uint16_t temp = ( clink->buffer[3] << 8 ) | clink->buffer[2];
	
	return temp;
}


/* requests and returns single data values depending on channel */
static int get_data(struct clink_device *clink, int command, int channel, bool two_byte_data)
{
	int ret;

	mutex_lock(&clink->mutex);

	ret = send_usb_cmd(clink, command, channel, 0, 0);
	if (ret)
		goto out_unlock;

	ret = clink->buffer[1];
	if (two_byte_data)
		ret = (ret << 8) + clink->buffer[2];

out_unlock:
	mutex_unlock(&clink->mutex);
	return ret;
}

static int clink_read_string(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, const char **str)
{
	struct clink_device *clink = dev_get_drvdata(dev);

	switch (type) {
		case hwmon_in:
			switch (attr) {
			case hwmon_in_label:
				*str = channel_labels[channel];
				return 0;
			default:
				break;
			}
			break;
		
		case hwmon_power:
			switch (attr) {
			case hwmon_power_label:
				*str = channel_labels[channel];
				return 0;
			default:
				break;
			}
			break;
			
		case hwmon_curr:
			switch (attr) {
				case hwmon_curr_label:
					*str = channel_labels[channel];
					return 0;
				default:
					break;
			}
			break;
				

		default:
			break;
	}

	return -EOPNOTSUPP;
}

static int clink_read(struct device *dev, enum hwmon_sensor_types type,
		    u32 attr, int channel, long *val)
{
	struct clink_device *clink = dev_get_drvdata(dev);
	int ret;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
            ret = rmi_temperature(clink, channel);
			if (ret < 0)
				return ret;
			*val = ret;
			return 0;
		default:
			break;
		}
		break;
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_input:
			ret = rmi_fan(clink);
			if (ret < 0)
				return ret;
			*val = ret;
			return 0;
		default:
			break;
		}
		break;
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
			ret = rmi_current(clink, channel);
			if (ret < 0)
				return ret;
			*val = ret;
			return 0;
		default:
			break;
		}
		break;
	case hwmon_power:
		switch (attr) {
		case hwmon_power_input:
			ret = rmi_power(clink, channel);
			if (ret < 0)
				return ret;
			*val = ret;
			return 0;
		default:
			break;
		}
		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
            ret = rmi_voltage(clink, channel);
			if (ret < 0)
				return ret;
			*val = ret;
			return 0;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
};

static umode_t clink_is_visible(const void *data, enum hwmon_sensor_types type,
			      u32 attr, int channel)
{
    return 0444;
};

static const struct hwmon_ops clink_hwmon_ops = {
	.is_visible = clink_is_visible,
	.read = clink_read,
	.read_string = clink_read_string,
};

static const struct hwmon_channel_info *corsairlink_info[] = {
    HWMON_CHANNEL_INFO(chip,
			   HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT,
			   HWMON_T_INPUT
			   ),
	HWMON_CHANNEL_INFO(fan,
               HWMON_F_LABEL | HWMON_F_INPUT
			   ),
	HWMON_CHANNEL_INFO(in,
               HWMON_I_LABEL|HWMON_I_INPUT,
			   HWMON_I_LABEL|HWMON_I_INPUT,
               HWMON_I_LABEL|HWMON_I_INPUT,
               HWMON_I_LABEL|HWMON_I_INPUT
			   ),
	HWMON_CHANNEL_INFO(curr,
			   HWMON_C_LABEL|HWMON_C_INPUT,
               HWMON_C_LABEL|HWMON_C_INPUT,
               HWMON_C_LABEL|HWMON_C_INPUT
			   ),
	HWMON_CHANNEL_INFO(power,
               HWMON_P_LABEL|HWMON_P_INPUT,
			   HWMON_P_LABEL|HWMON_P_INPUT,
               HWMON_P_LABEL|HWMON_P_INPUT,
               HWMON_P_LABEL|HWMON_P_INPUT
			   ),
	NULL
};

static const struct hwmon_chip_info clink_chip_info = {
	.ops = &clink_hwmon_ops,
	.info = corsairlink_info,
};

static int corsairlink_rmi_name(
    struct clink_device* clink,
    char* name,
    uint8_t name_size )
{
    int ret;

    clink->buffer[0] = 0xfe;
    clink->buffer[1] = 0x03;

    rmi_send_cmd(clink, true);

    memcpy(name, clink->buffer + 2, 16);

    return 0;
}

static int clink_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct clink_device *clink;
	int ret;

    printk("Start probing");
	clink = devm_kzalloc(&hdev->dev, sizeof(*clink), GFP_KERNEL);
	if (!clink)
		return -ENOMEM;

	clink->buffer = devm_kmalloc(&hdev->dev, OUT_BUFFER_SIZE, GFP_KERNEL);
	if (!clink->buffer)
		return -ENOMEM;

	ret = hid_parse(hdev);
	if (ret)
		return ret;

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret)
		return ret;

	ret = hid_hw_open(hdev);
	if (ret)
		goto out_hw_stop;

	clink->hdev = hdev;
	hid_set_drvdata(hdev, clink);
	mutex_init(&clink->mutex);
	init_completion(&clink->wait_input_report);

	hid_device_io_start(hdev);

    char name[64] = {0};
    ret = corsairlink_rmi_name(clink, name, 64);
    printk("Name:%s", name);
	/* temp and fan connection status only updates when device is powered on */
	//ret = get_temp_cnct(clink);
	if (ret)
		goto out_hw_close;

/*	ret = get_fan_cnct(clink);
	if (ret)
		goto out_hw_close;*/
	clink->hwmon_dev = hwmon_device_register_with_info(&hdev->dev, "corsairlink",
							 clink, &clink_chip_info, 0);
	if (IS_ERR(clink->hwmon_dev)) {
		ret = PTR_ERR(clink->hwmon_dev);
		goto out_hw_close;
	}

	return 0;

out_hw_close:
	hid_hw_close(hdev);
out_hw_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void clink_remove(struct hid_device *hdev)
{
	struct clink_device *clink = hid_get_drvdata(hdev);

	hwmon_device_unregister(clink->hwmon_dev);
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static const struct hid_device_id clink_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, 0x1c09) }, /* RM550i */
    { HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, 0x1c0a) }, /* RM650i */
    { HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, 0x1c0b) }, /* RM750i */
    { HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, 0x1c0c) }, /* RM850i */
    { HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, 0x1c0d) }, /* RM1000i */
    { HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, 0x1c03) }, /* HX550i */
    { HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, 0x1c04) }, /* HX650i */
    { HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, 0x1c05) }, /* HX750i */
    { HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, 0x1c06) }, /* HX850i */
    { HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, 0x1c07) }, /* HX1000i */
    { HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, 0x1c08) }, /* HX1200i */
	{ }
};

static struct hid_driver clink_driver = {
	.name = "corsair-link",
	.id_table = clink_devices,
	.probe = clink_probe,
	.remove = clink_remove,
	.raw_event = clink_raw_event,
};

MODULE_DEVICE_TABLE(hid, clink_devices);
MODULE_LICENSE("GPL");

static int __init clink_init(void)
{
	return hid_register_driver(&clink_driver);
}

static void __exit clink_exit(void)
{
	hid_unregister_driver(&clink_driver);
}

/*
 * When compiling this driver as built-in, hwmon initcalls will get called before the
 * hid driver and this driver would fail to register. late_initcall solves this.
 */
late_initcall(clink_init);
module_exit(clink_exit);
