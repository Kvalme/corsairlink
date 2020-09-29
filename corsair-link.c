// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * corsair-cpro.c - Linux driver for Corsair Commander Pro
 * Copyright (C) 2020 Marius Zachmann <mail@mariuszachmann.de>
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

#define USB_VENDOR_ID_CORSAIR			0x1b1c
#define USB_PRODUCT_ID_CORSAIR_HX1000i		0x1c07

#define OUT_BUFFER_SIZE		64
#define IN_BUFFER_SIZE		64
#define LABEL_LENGTH		16
#define REQ_TIMEOUT		300

#define CMD_GET_ID  0x00

#define CTL_GET_TMP_CNCT	0x10	/*
					 * returns in bytes 1-4 for each temp sensor:
					 * 0 not connected
					 * 1 connected
					 */
#define CTL_GET_TMP		0x11	/*
					 * send: byte 1 is channel, rest zero
					 * rcv:  returns temp for channel in centi-degree celsius
					 * in bytes 1 and 2
					 * returns 0x11 in byte 0 if no sensor is connected
					 */
#define CTL_GET_VOLT		0x12	/*
					 * send: byte 1 is rail number: 0 = 12v, 1 = 5v, 2 = 3.3v
					 * rcv:  returns millivolt in bytes 1,2
					 * returns error 0x10 if request is invalid
					 */
#define CTL_GET_FAN_CNCT	0x20	/*
					 * returns in bytes 1-6 for each fan:
					 * 0 not connected
					 * 1 3pin
					 * 2 4pin
					 */
#define CTL_GET_FAN_RPM		0x21	/*
					 * send: byte 1 is channel, rest zero
					 * rcv:  returns rpm in bytes 1,2
					 */
#define CTL_GET_FAN_PWM		0x22	/*
					 * send: byte 1 is channel, rest zero
					 * rcv:  returns pwm in byte 1 if it was set
					 *	 returns error 0x12 if fan is controlled via
					 *	 fan_target or fan curve
					 */
#define CTL_SET_FAN_FPWM	0x23	/*
					 * set fixed pwm
					 * send: byte 1 is fan number
					 * send: byte 2 is percentage from 0 - 100
					 */
#define CTL_SET_FAN_TARGET	0x24	/*
					 * set target rpm
					 * send: byte 1 is fan number
					 * send: byte 2-3 is target
					 * device accepts all values from 0x00 - 0xFFFF
					 */

#define NUM_FANS            1
#define NUM_TEMP_SENSORS    2
#define NUM_POWER_SOURCES   4

struct ccp_device {
	struct hid_device *hdev;
	struct device *hwmon_dev;
	struct completion wait_input_report;
	struct mutex mutex; /* whenever buffer is used, lock before send_usb_cmd */
	u8 *buffer;
	int target[6];
	char fan_label[6][LABEL_LENGTH];
	
};

static const char channel_labels[4][LABEL_LENGTH] = { "Power supply", "+12V", "+5V", "+3.3V"};

/* converts response error in buffer to errno */
static int ccp_get_errno(struct ccp_device *ccp)
{
	switch (ccp->buffer[0]) {
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
		hid_dbg(ccp->hdev, "unknown device response error: %d", ccp->buffer[0]);
		return -EIO;
	}
}

/* send command, check for error in response, response in ccp->buffer */
static int send_usb_cmd(struct ccp_device *ccp, u8 command, u8 byte1, u8 byte2, u8 byte3)
{
	unsigned long t;
	int ret;

	memset(ccp->buffer, 0x00, OUT_BUFFER_SIZE);
	ccp->buffer[0] = command;
	ccp->buffer[1] = byte1;
	ccp->buffer[2] = byte2;
	ccp->buffer[3] = byte3;

	reinit_completion(&ccp->wait_input_report);

	ret = hid_hw_output_report(ccp->hdev, ccp->buffer, OUT_BUFFER_SIZE);
	if (ret < 0)
		return ret;

	t = wait_for_completion_timeout(&ccp->wait_input_report, msecs_to_jiffies(REQ_TIMEOUT));
	if (!t)
		return -ETIMEDOUT;

	return ccp_get_errno(ccp);
}

static int ccp_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *data, int size)
{
	struct ccp_device *ccp = hid_get_drvdata(hdev);

	/* only copy buffer when requested */
	if (completion_done(&ccp->wait_input_report))
		return 0;

	memcpy(ccp->buffer, data, min(IN_BUFFER_SIZE, size));
	complete(&ccp->wait_input_report);

	return 0;
}

static int rmi_send_cmd(struct ccp_device* ccp, bool wait)
{
    int ret = 0;

    reinit_completion(&ccp->wait_input_report);

    ret = hid_hw_output_report(ccp->hdev, ccp->buffer, OUT_BUFFER_SIZE);
    if (ret < 0)
        return ret;

    if (wait)
    {
        ret = wait_for_completion_timeout(&ccp->wait_input_report, msecs_to_jiffies(REQ_TIMEOUT));
        if (!ret)
            return -ETIMEDOUT;
    }

    return ret;
}

static int rmi_temperature(
    struct ccp_device* ccp,
    uint8_t probe)
{
    int ret;
    uint16_t temp;

    ccp->buffer[0] = 0x03;
    ccp->buffer[1] = 0x8D + probe;
    ccp->buffer[2] = 0x00;
    ccp->buffer[3] = 0x00;

    ret = rmi_send_cmd(ccp, true);
    if (ret < 0)
        return ret;

    temp = ( ccp->buffer[2] << 8 ) + ccp->buffer[3];

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
    struct ccp_device* ccp,
    uint8_t probe)
{
    int ret;

    if (probe == 0) //Power suuply
    {
        ccp->buffer[0] = 0x03;
        ccp->buffer[1] = 0x88;
        ccp->buffer[2] = 0x00;
        ccp->buffer[3] = 0x00;
    }
    else
    {

        ccp->buffer[0] = 0x02;
        ccp->buffer[1] = 0x00;
        ccp->buffer[2] = probe - 1;

        rmi_send_cmd(ccp, true);

        ccp->buffer[0] = 0x03;
        ccp->buffer[1] = 0x8B;
        ccp->buffer[2] = 0x00;
        ccp->buffer[3] = 0x00;
    }


    ret = rmi_send_cmd(ccp, true);
    if (ret < 0)
        return ret;

	uint16_t temp = ( ccp->buffer[3] << 8 ) | ccp->buffer[2];

	return get_int_from_uint16_double(temp);
}

static int rmi_power(
    struct ccp_device* ccp,
    uint8_t probe)
{
    int ret;

    if (probe == 0) //Power suuply
    {
        ccp->buffer[0] = 0x03;
        ccp->buffer[1] = 0xEE;
        ccp->buffer[2] = 0x00;
        ccp->buffer[3] = 0x00;
    }
    else
    {

        ccp->buffer[0] = 0x02;
        ccp->buffer[1] = 0x00;
        ccp->buffer[2] = probe - 1;

        rmi_send_cmd(ccp, true);

        ccp->buffer[0] = 0x03;
        ccp->buffer[1] = 0x96;
        ccp->buffer[2] = 0x00;
        ccp->buffer[3] = 0x00;
    }


    ret = rmi_send_cmd(ccp, true);
    if (ret < 0)
        return ret;

    uint16_t temp = ( ccp->buffer[3] << 8 ) | ccp->buffer[2];

	return get_int_from_uint16_double(temp) * 1000;
}

static int rmi_current(
	struct ccp_device* ccp,
	uint8_t probe)
{
	int ret;
	
	ccp->buffer[0] = 0x02;
	ccp->buffer[1] = 0x00;
	ccp->buffer[2] = probe;
	
	rmi_send_cmd(ccp, true);
	
	ccp->buffer[0] = 0x03;
	ccp->buffer[1] = 0x8C;
	ccp->buffer[2] = 0x00;
	ccp->buffer[3] = 0x00;
	
	ret = rmi_send_cmd(ccp, true);
	if (ret < 0)
		return ret;
	
	uint16_t temp = ( ccp->buffer[3] << 8 ) | ccp->buffer[2];
	
	return get_int_from_uint16_double(temp);
}

static int rmi_fan(
	struct ccp_device* ccp)
{
	int ret;
	
	ccp->buffer[0] = 0x03;
	ccp->buffer[1] = 0x90;
	ccp->buffer[2] = 0x00;
	ccp->buffer[3] = 0x00;
	
	ret = rmi_send_cmd(ccp, true);
	if (ret < 0)
		return ret;
	
	uint16_t temp = ( ccp->buffer[3] << 8 ) | ccp->buffer[2];
	
	return temp;
}


/* requests and returns single data values depending on channel */
static int get_data(struct ccp_device *ccp, int command, int channel, bool two_byte_data)
{
	int ret;

	mutex_lock(&ccp->mutex);

	ret = send_usb_cmd(ccp, command, channel, 0, 0);
	if (ret)
		goto out_unlock;

	ret = ccp->buffer[1];
	if (two_byte_data)
		ret = (ret << 8) + ccp->buffer[2];

out_unlock:
	mutex_unlock(&ccp->mutex);
	return ret;
}

static int ccp_read_string(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, const char **str)
{
	struct ccp_device *ccp = dev_get_drvdata(dev);

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

static int ccp_read(struct device *dev, enum hwmon_sensor_types type,
		    u32 attr, int channel, long *val)
{
	struct ccp_device *ccp = dev_get_drvdata(dev);
	int ret;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
            ret = rmi_temperature(ccp, channel);
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
			ret = rmi_fan(ccp);
			if (ret < 0)
				return ret;
			*val = ret;
			return 0;
		case hwmon_fan_target:
			/* how to read target values from the device is unknown */
			/* driver returns last set value or 0			*/
			if (ccp->target[channel] < 0)
				return -ENODATA;
			*val = ccp->target[channel];
			return 0;
		default:
			break;
		}
		break;
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
			ret = rmi_current(ccp, channel);
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
			ret = rmi_power(ccp, channel);
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
            ret = rmi_voltage(ccp, channel);
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

static int ccp_write(struct device *dev, enum hwmon_sensor_types type,
		     u32 attr, int channel, long val)
{
/*	struct ccp_device *ccp = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
		default:
			break;
		}
		break;
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_target:
		default:
			break;
		}
	default:
		break;
	}*/

	return -EOPNOTSUPP;
};

static umode_t ccp_is_visible(const void *data, enum hwmon_sensor_types type,
			      u32 attr, int channel)
{
    return 0444;
};

static const struct hwmon_ops ccp_hwmon_ops = {
	.is_visible = ccp_is_visible,
	.read = ccp_read,
	.read_string = ccp_read_string,
	.write = ccp_write,
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

static const struct hwmon_chip_info ccp_chip_info = {
	.ops = &ccp_hwmon_ops,
	.info = corsairlink_info,
};

static int corsairlink_rmi_name(
    struct ccp_device* ccp,
    char* name,
    uint8_t name_size )
{
    int ret;

    ccp->buffer[0] = 0xfe;
    ccp->buffer[1] = 0x03;

    rmi_send_cmd(ccp, true);

    memcpy(name, ccp->buffer + 2, 16);

    return 0;
}

static int ccp_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct ccp_device *ccp;
	int ret;

    printk("Start probing");
	ccp = devm_kzalloc(&hdev->dev, sizeof(*ccp), GFP_KERNEL);
	if (!ccp)
		return -ENOMEM;

	ccp->buffer = devm_kmalloc(&hdev->dev, OUT_BUFFER_SIZE, GFP_KERNEL);
	if (!ccp->buffer)
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

	ccp->hdev = hdev;
	hid_set_drvdata(hdev, ccp);
	mutex_init(&ccp->mutex);
	init_completion(&ccp->wait_input_report);

	hid_device_io_start(hdev);

    char name[64] = {0};
    ret = corsairlink_rmi_name(ccp, name, 64);
    printk("Name:%s", name);
	/* temp and fan connection status only updates when device is powered on */
	//ret = get_temp_cnct(ccp);
	if (ret)
		goto out_hw_close;

/*	ret = get_fan_cnct(ccp);
	if (ret)
		goto out_hw_close;*/
	ccp->hwmon_dev = hwmon_device_register_with_info(&hdev->dev, "corsairlink",
							 ccp, &ccp_chip_info, 0);
	if (IS_ERR(ccp->hwmon_dev)) {
		ret = PTR_ERR(ccp->hwmon_dev);
		goto out_hw_close;
	}

	return 0;

out_hw_close:
	hid_hw_close(hdev);
out_hw_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void ccp_remove(struct hid_device *hdev)
{
	struct ccp_device *ccp = hid_get_drvdata(hdev);

	hwmon_device_unregister(ccp->hwmon_dev);
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static const struct hid_device_id ccp_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_CORSAIR, USB_PRODUCT_ID_CORSAIR_HX1000i) },
	{ }
};

static struct hid_driver ccp_driver = {
	.name = "corsair-link",
	.id_table = ccp_devices,
	.probe = ccp_probe,
	.remove = ccp_remove,
	.raw_event = ccp_raw_event,
};

MODULE_DEVICE_TABLE(hid, ccp_devices);
MODULE_LICENSE("GPL");

static int __init ccp_init(void)
{
	return hid_register_driver(&ccp_driver);
}

static void __exit ccp_exit(void)
{
	hid_unregister_driver(&ccp_driver);
}

/*
 * When compiling this driver as built-in, hwmon initcalls will get called before the
 * hid driver and this driver would fail to register. late_initcall solves this.
 */
late_initcall(ccp_init);
module_exit(ccp_exit);
