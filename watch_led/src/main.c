/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/counter.h>
#include <drivers/i2c.h>
#include <string.h>
#include <shell/shell.h>

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE	DT_ALIAS(led0)
#define LED1_NODE	DT_ALIAS(led1)
#define LED2_NODE	DT_ALIAS(led2)
#define LED3_NODE	DT_ALIAS(led3)
#define LED4_NODE	DT_ALIAS(led4)
#define LED5_NODE	DT_ALIAS(led5)
#define LED6_NODE	DT_ALIAS(led6)
#define LED7_NODE	DT_ALIAS(led7)
#define LED8_NODE	DT_ALIAS(led8)
#define LED9_NODE	DT_ALIAS(led9)
#define LED10_NODE	DT_ALIAS(led10)
#define LED11_NODE	DT_ALIAS(led11)
#define LEDA_NODE	DT_ALIAS(leda)
#define LEDB_NODE	DT_ALIAS(ledb)
#define LEDC_NODE	DT_ALIAS(ledc)
#define LEDD_NODE	DT_ALIAS(ledd)
#define LEDE_NODE	DT_ALIAS(lede)
#define LEDF_NODE	DT_ALIAS(ledf)
#define LEDG_NODE	DT_ALIAS(ledg)
#define LEDDP_NODE	DT_ALIAS(leddp)
#define LEDSEC_NODE	DT_ALIAS(ledsec)

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay)
#define I2C_DEV_NAME	DT_LABEL(DT_NODELABEL(i2c1))
#else
#error "Please set the correct I2C device"
#endif

#define DELAY		1000000

#define CLEAR_7SEG	0xFF0000
#define ON_7SEG		1
#define OFF_7SEG	0
#define LED_MIN		0
#define LED_HOUR	1
#define LED_DATE	2
#define LED_MONTH	3
#define LED_YEAR	4

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   10
/* 1 msec = 1000 usec */
#define WAIT_TIME_US   10

#define DS3231_I2C_ADDR	0x68
#define I2C_BITRATE_STANDARD	100000	/* 100 Kbit/s */
#define I2C_BITRATE_FAST	400000	/* 400 Kbit/s */

#define INIT_PREEMPT_PRIO 1
#define INIT_PREEMPT_OPTION (K_USER | K_INHERIT_PERMS)
#define STACK_SIZE	500
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);
static const struct gpio_dt_spec led4 = GPIO_DT_SPEC_GET(LED4_NODE, gpios);
static const struct gpio_dt_spec led5 = GPIO_DT_SPEC_GET(LED5_NODE, gpios);
static const struct gpio_dt_spec led6 = GPIO_DT_SPEC_GET(LED6_NODE, gpios);
static const struct gpio_dt_spec led7 = GPIO_DT_SPEC_GET(LED7_NODE, gpios);
static const struct gpio_dt_spec led8 = GPIO_DT_SPEC_GET(LED8_NODE, gpios);
static const struct gpio_dt_spec led9 = GPIO_DT_SPEC_GET(LED9_NODE, gpios);
static const struct gpio_dt_spec led10 = GPIO_DT_SPEC_GET(LED10_NODE, gpios);
static const struct gpio_dt_spec led11 = GPIO_DT_SPEC_GET(LED11_NODE, gpios);
static const struct gpio_dt_spec leda = GPIO_DT_SPEC_GET(LEDA_NODE, gpios);
static const struct gpio_dt_spec ledb = GPIO_DT_SPEC_GET(LEDB_NODE, gpios);
static const struct gpio_dt_spec ledc = GPIO_DT_SPEC_GET(LEDC_NODE, gpios);
static const struct gpio_dt_spec ledd = GPIO_DT_SPEC_GET(LEDD_NODE, gpios);
static const struct gpio_dt_spec lede = GPIO_DT_SPEC_GET(LEDE_NODE, gpios);
static const struct gpio_dt_spec ledf = GPIO_DT_SPEC_GET(LEDF_NODE, gpios);
static const struct gpio_dt_spec ledg = GPIO_DT_SPEC_GET(LEDG_NODE, gpios);
static const struct gpio_dt_spec ledsec = GPIO_DT_SPEC_GET(LEDSEC_NODE, gpios);

struct gpio_dt_spec led[12] = {led0, led1, led2, led3, led4, led5, led6, led7, led8, led9, led10, led11};

static const uint32_t decode_7seg[10] = {0x000300FC, 0x00D70028, 0x000D00F2, 0x001500EA, 0x00D1002E,	\
										 0x003100CE, 0x002100DE, 0x005700A8, 0x000100FE,0x401100EE};
const struct device *port_7seg;
const struct device *counter_dev;
const struct device *i2c_dev;

static uint8_t time_sec, time_min, time_hour, time_day, time_date, time_month, time_year;

static void thread_entry_7seg(void *p1, void *p2, void *p3);
static void thread_i2c_ds3231(void *p1, void *p2, void *p3);

K_THREAD_DEFINE(thread_1, STACK_SIZE, thread_entry_7seg,
				NULL, NULL, NULL, INIT_PREEMPT_PRIO, 2, 0);

K_THREAD_DEFINE(thread_3, STACK_SIZE, thread_i2c_ds3231,
				NULL, NULL, NULL, INIT_PREEMPT_PRIO, 0, 0);

static void led_7seg_turn_on(uint8_t num_led)
{

	for (size_t i = 0; i < 12; i++) {
		if (num_led == i) {
			gpio_pin_set(led[i].port, led[i].pin, ON_7SEG);
		}
		else
			gpio_pin_set(led[i].port, led[i].pin, OFF_7SEG);
	}
	
}

static int led_7seg_display(uint8_t cluster, uint8_t num)
{
	if (num >= 100) {
		return ENOTSUP;
	}
	// printk("7-segment display: %d\n", num);
	if (cluster == LED_MIN) {
		/* First number */
		led_7seg_turn_on(10);
		gpio_port_set_bits_raw(port_7seg, CLEAR_7SEG);
		gpio_port_set_bits_raw(port_7seg, decode_7seg[num / 10]);
		k_usleep(WAIT_TIME_US);
		/* Second number */
		led_7seg_turn_on(11);
		gpio_port_set_bits_raw(port_7seg, CLEAR_7SEG);
		gpio_port_set_bits_raw(port_7seg, decode_7seg[num % 10]);
		k_usleep(WAIT_TIME_US);
	}
	else if (cluster == LED_HOUR) {
		/* First number */
		led_7seg_turn_on(8);
		gpio_port_set_bits_raw(port_7seg, CLEAR_7SEG);
		gpio_port_set_bits_raw(port_7seg, decode_7seg[num / 10]);
		k_usleep(WAIT_TIME_US);
		/* Second number */
		led_7seg_turn_on(9);
		gpio_port_set_bits_raw(port_7seg, CLEAR_7SEG);
		gpio_port_set_bits_raw(port_7seg, decode_7seg[num % 10]);
		k_usleep(WAIT_TIME_US);
	}
	else if (cluster == LED_DATE) {
		/* First number */
		led_7seg_turn_on(7);
		gpio_port_set_bits_raw(port_7seg, CLEAR_7SEG);
		gpio_port_set_bits_raw(port_7seg, decode_7seg[num / 10]);
		k_usleep(WAIT_TIME_US);
		/* Second number */
		led_7seg_turn_on(6);
		gpio_port_set_bits_raw(port_7seg, CLEAR_7SEG);
		gpio_port_set_bits_raw(port_7seg, decode_7seg[num % 10]);
		k_usleep(WAIT_TIME_US);
	}
	else if (cluster == LED_MONTH) {
		/* First number */
		led_7seg_turn_on(2);
		gpio_port_set_bits_raw(port_7seg, CLEAR_7SEG);
		gpio_port_set_bits_raw(port_7seg, decode_7seg[num / 10]);
		k_usleep(WAIT_TIME_US);
		/* Second number */
		led_7seg_turn_on(3);
		gpio_port_set_bits_raw(port_7seg, CLEAR_7SEG);
		gpio_port_set_bits_raw(port_7seg, decode_7seg[num % 10]);
		k_usleep(WAIT_TIME_US);
	}
	else if (cluster == LED_YEAR) {
		/* First number */
		led_7seg_turn_on(4);
		gpio_port_set_bits_raw(port_7seg, CLEAR_7SEG);
		gpio_port_set_bits_raw(port_7seg, decode_7seg[2]);
		k_usleep(WAIT_TIME_US);
		/* Second number */
		led_7seg_turn_on(5);
		gpio_port_set_bits_raw(port_7seg, CLEAR_7SEG);
		gpio_port_set_bits_raw(port_7seg, decode_7seg[0]);
		k_usleep(WAIT_TIME_US);

		led_7seg_turn_on(0);
		gpio_port_set_bits_raw(port_7seg, CLEAR_7SEG);
		gpio_port_set_bits_raw(port_7seg, decode_7seg[num / 10]);
		k_usleep(WAIT_TIME_US);

		led_7seg_turn_on(1);
		gpio_port_set_bits_raw(port_7seg, CLEAR_7SEG);
		gpio_port_set_bits_raw(port_7seg, decode_7seg[num % 10]);
		k_usleep(WAIT_TIME_US);
	}
	else {
		return -EINVAL;
	}
	return 0;
}

static void thread_entry_7seg(void *p1, void *p2, void *p3)
{
	if (!device_is_ready(ledb.port)) {
		return;
	}

	static uint8_t tem_sec_old = 0;
	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led4, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led5, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led6, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led7, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led8, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led9, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led10, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led11, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&ledsec, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&leda, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&ledb, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&ledc, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&ledd, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&lede, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&ledf, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&ledg, GPIO_OUTPUT_INACTIVE);

	gpio_pin_set(led0.port, led0.pin, OFF_7SEG);
	gpio_pin_set(led1.port, led1.pin, OFF_7SEG);
	gpio_pin_set(led2.port, led2.pin, OFF_7SEG);
	gpio_pin_set(led3.port, led3.pin, OFF_7SEG);
	gpio_pin_set(led4.port, led4.pin, OFF_7SEG);
	gpio_pin_set(led5.port, led5.pin, OFF_7SEG);
	gpio_pin_set(led6.port, led6.pin, OFF_7SEG);
	gpio_pin_set(led7.port, led7.pin, OFF_7SEG);
	gpio_pin_set(led8.port, led8.pin, OFF_7SEG);
	gpio_pin_set(led9.port, led9.pin, OFF_7SEG);
	gpio_pin_set(led10.port, led10.pin, OFF_7SEG);
	gpio_pin_set(led11.port, led11.pin, OFF_7SEG);
	gpio_pin_set(ledsec.port, ledsec.pin, OFF_7SEG);
	port_7seg = led8.port;

	while (1) {
		if (tem_sec_old != time_sec) {
			gpio_pin_toggle(ledsec.port, ledsec.pin);
			tem_sec_old = time_sec;
		}

		led_7seg_display(LED_MIN, time_min);
		led_7seg_display(LED_HOUR, time_hour);
		led_7seg_display(LED_DATE, time_date);
		led_7seg_display(LED_MONTH, time_month);
		led_7seg_display(LED_YEAR, time_year);
		k_msleep(2);
	}
}

uint8_t DECtoBCD(uint8_t DEC)
{
    uint8_t reg_low, reg_hight;

	reg_low = DEC % 10;
    reg_hight = (DEC / 10) << 4;

    return(reg_hight + reg_low);
}

uint8_t BCDtoDEC(uint8_t BCD)
{
    uint8_t reg_low, reg_hight;

    reg_low = BCD & 0x0F;
    reg_hight = (BCD >> 4) * 10;

    return(reg_hight + reg_low);
}

static void thread_i2c_ds3231(void *p1, void *p2, void *p3)
{
	int ret;
	uint8_t data[7];

	while (1) {
		(void)memset(data, 0, sizeof(data));
		ret = i2c_burst_read(i2c_dev, DS3231_I2C_ADDR, 0x00, data, sizeof(data));
		if (ret) {
			printk("Error reading time from DS3231! error code (%d)\n", ret);
		}
		else {
			time_sec = BCDtoDEC(data[0]);
			time_min = BCDtoDEC(data[1]);
			time_hour = BCDtoDEC(data[2]);
			time_day = BCDtoDEC(data[3]);
			time_date = BCDtoDEC(data[4]);
			time_month = BCDtoDEC(data[5]);
			time_year = BCDtoDEC(data[6]);
		}

		k_msleep(200);
	}
}

#ifdef CONFIG_SHELL
static int cmd_ds3231_time(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	char *day_arr[6] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

	shell_print(shell, "Day: %s", day_arr[time_day - 1]);
	shell_print(shell, "Time: %zd : %zd : %zd", time_hour, time_min, time_sec);
	shell_print(shell, "Date: %zd : %zd : 20%zd", time_date, time_month, time_year);

	return 0;
}


static int cmd_ds3231_set_time(const struct shell *shell, size_t argc, char **argv)
{
	char *day_arr[6] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
	// for (size_t cnt = 0; cnt < argc; cnt++) {
	// 	if (!strcmp(argv[cnt], '-p')) {
	// 		int min = convert_string_to_int(argv[cnt]);
	// 		if (i2c_burst_write(i2c_dev, DS3231_I2C_ADDR, 0x01, min, 1)) {
	// 			printk("Fail to write\n");
	// 		}
	// 	}
	// 	else if (!strcmp(argv[cnt], '-g')) {
	// 		int hour = convert_string_to_int(argv[cnt + 1]);
	// 		if (i2c_burst_write(i2c_dev, DS3231_I2C_ADDR, 0x02, hour, 1)) {
	// 			printk("Fail to write\n");
	// 		}
	// 	}
	// 	else if (!strcmp(argv[cnt], '-d')) {
	// 		int day = convert_string_to_int(argv[cnt + 1]);
	// 		if (i2c_burst_write(i2c_dev, DS3231_I2C_ADDR, 0x04, day, 1)) {
	// 			printk("Fail to write\n");
	// 		}
	// 	}		
	// 	else if (!strcmp(argv[cnt], '-m')) {
	// 		int month = convert_string_to_int(argv[cnt + 1]);
	// 		if (i2c_burst_write(i2c_dev, DS3231_I2C_ADDR, 0x05, month, 1)) {
	// 			printk("Fail to write\n");
	// 		}
	// 	}
	// 	else if (!strcmp(argv[cnt], '-y')) {
	// 		int year = convert_string_to_int(argv[cnt + 1]);
	// 		if (i2c_burst_write(i2c_dev, DS3231_I2C_ADDR, 0x06, year, 1)) {
	// 			printk("Fail to write\n");
	// 		}
	// 	}
	// 	else
	// 		shell_print(shell, "  argv[%zd] = %s", cnt, argv[cnt]);	
	// }

	uint8_t data[7];
	(void)memset(data, 0, sizeof(data));
	data[0] = DECtoBCD(0);
	data[1] = DECtoBCD(50);
	data[2] = DECtoBCD(0);
	data[3] = DECtoBCD(4);
	data[4] = DECtoBCD(28);
	data[5] = DECtoBCD(4);
	data[6] = DECtoBCD(22);
	if (i2c_burst_write(i2c_dev, DS3231_I2C_ADDR, 0x00, data, sizeof(data))) {
		printk("Fail to write\n");
	}

	shell_print(shell, "Day: %s", day_arr[time_day - 1]);
	shell_print(shell, "Time: %zd : %zd : %zd", time_hour, time_min, time_sec);
	shell_print(shell, "Date: %zd/ %zd/ 20%zd", time_date, time_month, time_year);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_demo,
	SHELL_CMD(time, NULL, "Show time command.", cmd_ds3231_time),
	SHELL_CMD(set_time, NULL, "Show time command.", cmd_ds3231_set_time),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(ds3231, &sub_demo, "DS3231 commands", NULL);

#endif

void main(void)
{
	i2c_dev = device_get_binding(I2C_DEV_NAME);
	if (!device_is_ready(i2c_dev)) {
		printk("I2C: Device is not ready\n");
		return;
	}
	uint32_t i2c_cfg = I2C_SPEED_SET(I2C_BITRATE_FAST) | I2C_MODE_MASTER;
	i2c_configure(i2c_dev, i2c_cfg);
	printk("\t--HUST Traffic Lights--\n");
}
