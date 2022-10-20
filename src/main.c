#include <zephyr.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <stdio.h>
#include <logging/log.h>
#include "max30102.h"

#define interruptpin 20

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

LOG_MODULE_REGISTER(MAIN, CONFIG_LOG_DEFAULT_LEVEL);

K_FIFO_DEFINE(ppg_items);
K_SEM_DEFINE(sem_initialized_max30102, 0, 1);
K_SEM_DEFINE(sem_interruptmax30102, 0, 1);

const struct device *gpio0Port;	
struct gpio_callback interrupt_max30102;


typedef struct { 
    void *fifo_reserved;   /* 1st word reserved for use by FIFO */
    uint32_t ppgvalueRed;
	uint32_t ppgvalueIr;
} fifo_item_t;

void max30102isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {	
	k_sem_give(&sem_interruptmax30102);		
}

void attachinterrupt(void) {
	
	gpio0Port = DEVICE_DT_GET(DT_NODELABEL(gpio0));	
	__ASSERT(device_is_ready(gpio0Port), "port gpio0 not ready");
	gpio_pin_configure(gpio0Port, interruptpin, (GPIO_INPUT | GPIO_PULL_UP));
	gpio_pin_interrupt_configure(gpio0Port, interruptpin, GPIO_INT_EDGE_FALLING);
	gpio_init_callback(&interrupt_max30102, max30102isr, BIT(interruptpin));
  	gpio_add_callback(gpio0Port, &interrupt_max30102);
}

void main(void)
{	
	const struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	if (usb_enable(NULL)) {
		return;
	}

	/* Poll if the DTR flag was set */
	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		/* Give CPU resources to low priority threads. */
		k_sleep(K_MSEC(100));
	}

	
	if(max30102_init()) {
		LOG_ERR("could not initialize");
		k_sleep(K_FOREVER);		
	}	
	attachinterrupt();
	uint8_t max_read_state = max30102_interrupt_state(); //clear interrupt;

	LOG_INF("max initialized");
	k_sem_give(&sem_initialized_max30102);
	
	for(;;) {
		fifo_item_t *queued;
		queued = k_fifo_get(&ppg_items, K_FOREVER);
		#if (led_channels > 1)
			printk("%d,%d\n", queued->ppgvalueIr, queued->ppgvalueRed);
		#else
			printk("%d\n", queued->ppgvalueIr);
		#endif

	}
}


void max30102read(void) {	
	k_sem_take(&sem_initialized_max30102, K_FOREVER);
	
	uint8_t amount_samples;
	uint8_t isrstate;
	ppg_item_t buffer[32]; //maximum amount for ic
	while (1) {		
		k_sem_take(&sem_interruptmax30102, K_MSEC(5000));
		isrstate = max30102_interrupt_state();
		if (isrstate != 0b10000000) {
			LOG_WRN("unexpected max30102int: %d", isrstate);			
		} 
		amount_samples = GetSamplesAvailable();		
		
		max30102data(buffer, amount_samples);
		
		for (int i = 0; i < amount_samples; i++)
		{
			fifo_item_t toqueue = {.ppgvalueIr = buffer[i].ir, .ppgvalueRed = buffer[i].red};			
			k_fifo_put(&ppg_items, &toqueue);
		} 			
	}
}

K_THREAD_DEFINE(max30102_read_thread_id, 1024, max30102read, NULL, NULL,
		NULL, 7, 0, 0);
