/*
 * Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#define DEVICE_NAME		CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN		(sizeof(DEVICE_NAME) - 1)
#define BLE_READ_PIPE_STACK_SIZE 500
#define BLE_WRITE_PIPE_STACK_SIZE 1024
#define BLUE_LIGHT_THREAD_STACK_SIZE 500
// #define CONSOLE_INPUT_THREAD_STACK_SIZE 500
#define BLE_READ_PIPE_PRIORITY 5
#define BLE_WRITE_PIPE_PRIORITY 5
// #define CONSOLE_INPUT_THREAD_PRIORITY 5
#define MAX_PIPE_READ_SIZE 180
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define LED0_NODE DT_ALIAS(led0)

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

static void notif_enabled(bool enabled, void *ctx);
static void received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx);

K_THREAD_STACK_DEFINE(ble_read_pipe_stack_area, BLE_READ_PIPE_STACK_SIZE);
struct k_thread ble_read_pipe_thread_data;

K_THREAD_STACK_DEFINE(ble_write_pipe_stack_area, BLE_WRITE_PIPE_STACK_SIZE);
struct k_thread ble_write_pipe_thread_data;

K_THREAD_STACK_DEFINE(blue_light_thread_stack_area, BLUE_LIGHT_THREAD_STACK_SIZE);
struct k_thread blue_light_thread_data;


// K_THREAD_STACK_DEFINE(console_input_thread_stack_area, CONSOLE_INPUT_THREAD_STACK_SIZE);
// struct k_thread console_input_thread_data;


static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

struct bt_nus_cb nus_listener = {
    .notif_enabled = notif_enabled,
    .received = received,
};

struct bt_conn *default_conn = NULL;

uint8_t __aligned(4) ble_input_ring_buffer[MAX_PIPE_READ_SIZE];
struct k_pipe ble_input_pipe;

uint8_t __aligned(4) ble_output_ring_buffer[MAX_PIPE_READ_SIZE];
struct k_pipe ble_output_pipe;

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);  

static void notif_enabled(bool enabled, void *ctx)
{
    ARG_UNUSED(ctx);

    printk("%s() - %s\n", __func__, (enabled ? "Enabled" : "Disabled"));
}

static void received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(ctx);

    printk("%s() - Len: %d, Message: %.*s\n", __func__, len, len, (char *)data);
    int bytes_written = 0;
    uint8_t* data_cast = (uint8_t*)data;
    int res;

    while (bytes_written < len) {
        res = k_pipe_write(&ble_input_pipe, &data_cast[bytes_written],  len - bytes_written, K_FOREVER);
        bytes_written += res;
        printk("%s() write to pipe result %d\n", __func__, res);
        if (res <= 0)
            return;
    }
}

static void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        printk("Error connecting (0x%02x)\n", err);
        return;
    }
    char addr [BT_ADDR_LE_STR_LEN];
    default_conn = bt_conn_ref(conn);
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("Connected to %s\n", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
    printk("Disconnected - reason (0x%02x)\n", reason);
}

static void recycled(void) {
    int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err)
        printk("Advertising failed to start (err %d)\n", err);
    else
        printk("Advertising successfully restarted\n");
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .recycled = recycled,
};

static void ble_read_pipe_thread(void* a, void* b, void* c) {
    uint8_t data [MAX_PIPE_READ_SIZE] = {0};
    int bytes_read = 0;

    printk("Init %s\n", __func__);

    while(true) {
        bytes_read = k_pipe_read(&ble_input_pipe, data, MAX_PIPE_READ_SIZE, K_MSEC(500));
        if (bytes_read < 0) {
            k_sleep(K_MSEC(500));
            continue;
        }

        printk("%s() k_pipe_read = %d\n", __func__, bytes_read);

        printk("BLE READ : \"");
        for (int i = 0; i < bytes_read; i++) {
            printk("%c", data[i]);
        }
        printk("\"\n");
    }

}

static void ble_write_pipe_thread(void* a, void* b, void* c) {
    printk("Init %s\n", __func__);
    uint8_t data [MAX_PIPE_READ_SIZE] = {0};

    int bytes_read = 0;
    int err = -1;
    while(true) {
        bytes_read = k_pipe_read(&ble_output_pipe, data, MAX_PIPE_READ_SIZE, K_NO_WAIT);
        if (bytes_read <= 0) {
            k_sleep(K_MSEC(500));
            continue;
        }

        printk("%s(): k_pipe_read %d\n", __func__, bytes_read);
        if (!bytes_read)
            continue;

        printf("%s(): Data = \"", __func__);
        for (int i = 0; i < bytes_read; i++)
            printk("%c", data[i]);
        printk("\"\n");

        if (!default_conn)
            continue;
        
        err = bt_nus_send(default_conn, data, (uint16_t)bytes_read);
        if (err < 0)
            printk("%s(): bt_nus_send fail %d\n", __func__, err);

    }

}

static void serial_callback(const struct device *dev, void *user_data) {
    if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	// /* read until FIFO empty */
    uint8_t c;
    int res;
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        printk("%s() uart_fifo_read \"%c\"\n", __func__, c);
        res = k_pipe_write(&ble_output_pipe, &c, 1, K_NO_WAIT);
        printk("%s() k_pipe_write %d\n", __func__, res);
    }
}

static void blue_light_thread(void* a, void* b, void* c) {
    if (!gpio_is_ready_dt(&led))
		return;

	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    	if (ret < 0)
		return;

    while(true) {
        if (!default_conn) {
            gpio_pin_toggle_dt(&led);
            k_msleep(500);
        }
        else {
            gpio_pin_set_dt(&led, 1);
            k_sleep(K_MSEC(500));
        }
    }
}


int main(void)
{
   
    k_pipe_init(&ble_input_pipe, ble_input_ring_buffer, sizeof(ble_input_ring_buffer));
    k_pipe_init(&ble_output_pipe, ble_output_ring_buffer, sizeof(ble_output_ring_buffer));

    if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(uart_dev, serial_callback, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP)
			printk("Interrupt-driven UART API support not enabled\n");
		else if (ret == -ENOSYS)
			printk("UART device does not support interrupt-driven API\n");
		else
			printk("Error setting UART callback: %d\n", ret);
		return 0;
	}
	uart_irq_rx_enable(uart_dev);

    k_tid_t ble_read_thread_id = k_thread_create(&ble_read_pipe_thread_data, ble_read_pipe_stack_area,
                                 K_THREAD_STACK_SIZEOF(ble_read_pipe_stack_area),
                                 ble_read_pipe_thread,
                                 NULL, NULL, NULL,
                                 BLE_READ_PIPE_PRIORITY, 0, K_SECONDS(1));

    k_tid_t ble_write_thread_id = k_thread_create(&ble_write_pipe_thread_data, ble_write_pipe_stack_area,
                                 K_THREAD_STACK_SIZEOF(ble_write_pipe_stack_area),
                                 ble_write_pipe_thread,
                                 NULL, NULL, NULL,
                                 BLE_READ_PIPE_PRIORITY, 0, K_SECONDS(1));

    k_tid_t blue_light_thread_id = k_thread_create(&blue_light_thread_data, blue_light_thread_stack_area,
                                 K_THREAD_STACK_SIZEOF(blue_light_thread_stack_area),
                                 blue_light_thread,
                                 NULL, NULL, NULL,
                                 BLE_READ_PIPE_PRIORITY, 0, K_SECONDS(1));

    
    
    // k_tid_t console_read_thread_id = k_thread_create(&console_input_thread_data, console_input_thread_stack_area,
    //                              K_THREAD_STACK_SIZEOF(console_input_thread_stack_area),
    //                              console_input_thread,
    //                              NULL, NULL, NULL,
    //                              BLE_READ_PIPE_PRIORITY, 0, K_SECONDS(1));

    
    

    int err;
    err = bt_conn_cb_register(&conn_callbacks);
    if (err) {
        printk("Failed to register Connection callbacks: %d\n", err);
        return err;
    }

    err = bt_nus_cb_register(&nus_listener, NULL);
    if (err) {
        printk("Failed to register NUS callback: %d\n", err);
        return err;
    }

    err = bt_enable(NULL);
    if (err) {
        printk("Failed to enable bluetooth: %d\n", err);
        return err;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Failed to start advertising: %d\n", err);
        return err;
    }

    printk("Initialization complete\n");

    while (true) {
        k_sleep(K_MSEC(500));
    }

    return 0;
}
