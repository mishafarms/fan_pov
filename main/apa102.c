/*
 * apa102.cpp
 *
 *  Created on: Feb 14, 2017
 *      Author: micro
 */


/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "sdkconfig.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "pov_spi.h"
#include "driver/adc.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#define ESP_INTR_FLAG_DEFAULT 0

#if __cplusplus
extern "C" {
	void app_main();
	extern void timer_app_main();
	extern void gpio_app_main();
};
#else
void app_main();
extern void timer_app_main();
//extern void gpio_app_main();
#endif

/*
	This code will send the leds data to an APA102 string of leds in this case there are a total of 80 leds, 16 per
	fan blade. I will run them in 2 strings the first of which consists of 48 leds and the second of which will have
	32 leds to allow us to do all 80 leds in under 80us. this mean a jitter of about 1/6 of a pixel in the highest speed.
	I will use 2 SPI busses and have both using DMA. So the last interrupt will be from the string of 48 leds. I will send
	the first line to the first set of leds on the first string of leds, then line 20 to the second set of LEDS and line 40
	to the third set of LEDS on the first string. I will send line 60 to the first set of LEDS on the second string. and
	line 80 to the second set of LEDS on the second string. This will give the least amount of jitter.
	for each string of LEDS I need to send 4 bytes of 0 (can be 8) then the	leds data and then 4 extra bytes to clock all
	the data.
	We use lines 20 away from each other because we have 5 blades and I want 100 lines of 16 leds each. So since each
	fan blade is 1/5 of the way around from it's nearest neighbor, we must skip 100/5 or 20 lines. We must also wrap when
	we go past 100. So when the cycle trigger goes off, we figure out how long the cycle took and then subtract a small
	amount and divide by 100, that will be our line timer. Each time it fires we will have to send the next line of leds
	out to all 80 leds. So if our cycle (once around) is 55 ms, then our timer is 548us.
	I think that when the cycle interrupt comes we calculate the pixel timer and set a timer for a few microseconds. When
	that interrupt fires we start the SPI DMA and start our pixel timer. When the SPI DMA interrupt fires, we setup the
	next line. The next line will start it's DMA on the next pixel timer.

*/

//#define PIN_NUM_MISO GPIO_NUM_21
#define PIN_NUM_MOSI GPIO_NUM_13
#define PIN_NUM_CLK  GPIO_NUM_14
//#define PIN_NUM_CS   GPIO_NUM_22

#if 0
#define PIN_NUM_VMOSI GPIO_NUM_23
#define PIN_NUM_VCLK  GPIO_NUM_18
#else
#define PIN_NUM_VMOSI GPIO_NUM_5
#define PIN_NUM_VCLK  GPIO_NUM_16
#endif

#define PIN_NUM_DC   GPIO_NUM_25
#define PIN_NUM_DB   GPIO_NUM_26

#define PIN_NUM_CYCLE GPIO_NUM_12

spi_device_handle_t spi;
spi_device_handle_t vspi;

/*
 The ILI9341 needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void ili_spi_pre_transfer_callback(spi_transaction_t *t)
{
	gpio_num_t gpio_num = (gpio_num_t) t;

	if (gpio_num)
	{
		gpio_set_level(gpio_num, 1);
	}
}

void ili_spi_post_transfer_callback(spi_transaction_t *t)
{
	gpio_num_t gpio_num = (gpio_num_t) t;

	if (gpio_num)
	{
		gpio_set_level(gpio_num, 0);
	}
}

//Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
//function is finished because the SPI driver needs access to it even while we're already calculating the next line.

static spi_transaction_t trans[7];
static spi_transaction_t vspi_trans[7];

extern uint32_t leds[];
extern void cycle_isr(void* arg);

//Initialize the display
void ili_init(spi_device_handle_t spi)
{
    int x;

    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_DB, GPIO_MODE_OUTPUT);

    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = (gpio_int_type_t) GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1 << PIN_NUM_DB) | (1 << PIN_NUM_DC);
    //disable pull-down mode
    io_conf.pull_down_en = (gpio_pulldown_t) 0;
    //disable pull-up mode
    io_conf.pull_up_en = (gpio_pullup_t) 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(PIN_NUM_DB, 0);
    gpio_set_level(PIN_NUM_DC, 0);
//    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
//    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    // setup the pin to sample cycle times
#if 1
    gpio_set_direction(PIN_NUM_CYCLE, GPIO_MODE_INPUT);

    //interrupt on positive edge
    io_conf.intr_type = (gpio_int_type_t) GPIO_PIN_INTR_POSEDGE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_SEL_12;
    //disable pull-down mode
    io_conf.pull_down_en = (gpio_pulldown_t) 0;
    //disable pull-up mode
    io_conf.pull_up_en = (gpio_pullup_t) 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(PIN_NUM_CYCLE, cycle_isr, (void*) PIN_NUM_CYCLE);
#endif

    for (x=0; x<6; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        memset(&vspi_trans[x], 0, sizeof(spi_transaction_t));
    }

    // the first SPI bus has the first 3 blades of LEDs

    // setup the starting 32 bits of 0

    trans[0].flags = SPI_TRANS_DEFER_START;
    trans[0].length=32;
    trans[0].user = (void *) PIN_NUM_DB;
//    trans[0].tx_buffer = (void *) zeros;
    trans[0].next = &trans[1];

    for (x=1; x<4; x++)
    {
        trans[x].length=32 * 16;
        trans[x].next = &trans[x + 1];
    }

    trans[4].length=32;
//    trans[4].tx_buffer = (void *) two_off_leds;
    trans[4].next = (struct spi_transaction_t *) NULL;

    // do the same for vspi except we only have 2 blades of LEDS

    vspi_trans[0].flags = SPI_TRANS_DEFER_START;
    vspi_trans[0].length=32;
    vspi_trans[0].user = (void *) PIN_NUM_DC;	// use a differnt pin to debug
//    vspi_trans[0].tx_buffer = (void *) zeros;
    vspi_trans[0].next = &vspi_trans[1];

    for (x=1; x<3; x++)
    {
    	vspi_trans[x].length=32 * 16;
    	vspi_trans[x].next = &vspi_trans[x + 1];
    }

    vspi_trans[3].length=32;
//    vspi_trans[3].tx_buffer = (void *) two_off_leds;
    vspi_trans[3].next = (struct spi_transaction_t *) NULL;

    for (int y = 0; y < 100; y++)
    {
    	int bits = y;

    	for (x = 0; x < 8; x++)
    	{
    		uint8_t data;

    		// first set the high byte to 0xff as per the APA102 spec
//    		leds[(y * 16) + x] =  (0xff) | (y << 8) | (0xff << 16) | (x << 24);
    		if ( bits & 0x80)
    		{
    			data = 0x80;
    		}
    		else
    		{
    			data = 0x0;
    		}
    		bits <<= 1;

    		leds[(y * 16) + x] =  (0xff) | (data << 8) | (data << 16) | (data << 24);
    	}
    }
}

void db_prt(int len)
{
	printf("Total Len = %d\n", len);
}

void db_pulse(void)
{
	GPIO.out_w1ts = 1 << 25;
	GPIO.out_w1tc = 1 << 25;
}

void dc_pulse(void)
{
	GPIO.out_w1ts = 1 << 26;
	GPIO.out_w1tc = 1 << 26;
}

void db_on(void)
{
	GPIO.out_w1ts = 1 << 25;
}

void db_off(void)
{
	GPIO.out_w1tc = 1 << 25;
}

void db_prtn(int len, void *trans, void *next)
{
	printf("Total Len = %d trans = %p next = %p\n", len, trans, next);
}

//To send a line we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
//before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
//because the D/C line needs to be toggled in the middle.)
//This routine queues these commands up so they get sent as quickly as possible.

void send_all_lines(spi_device_handle_t p_spi, uint8_t lineNum)
{
    esp_err_t ret;
    int x;

    if (p_spi == spi)
    {
    	// we are doing 3 lines starting at the current line number

    	for (x=1; x<4; x++)
    	{
    		// starting at led 0 we have 4 bytes per led
    		trans[x].tx_buffer = (void *) (leds + (16 * ((lineNum + ((x - 1) * 20)) % 100)));
    	}

    	ret=pov_spi_device_queue_trans(p_spi, &trans[0], portMAX_DELAY);
        if(ret!=ESP_OK)
        {
        	printf("Error %d on pov_spi_device_queue_trans x = %d\n", ret, x);
        }
    }
    else if(p_spi == vspi)
    {
    	// we are doing 3 lines starting at the current line number

    	for (x=1; x<3; x++)
    	{
    		// starting at led 0 we have 4 bytes per led
    		vspi_trans[x].tx_buffer = (void *) (leds + (16 * ((lineNum + ((x + 2) * 20)) % 100)));
    	}

    	ret=pov_spi_device_queue_trans(p_spi, &vspi_trans[0], portMAX_DELAY);
        if(ret!=ESP_OK)
        {
        	printf("Error %d on pov_spi_device_queue_trans x = %d\n", ret, x);
        }
    }

    //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
    //mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
    //finish because we may as well spend the time calculating the next line. When that is done, we can call
    //send_line_finish, which will wait for the transfers to be done and check their status.
}


void send_line_finish(spi_device_handle_t spi)
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 6 transactions to be done and get back the results.
    for (int x=0; x<1; x++) {
        ret=pov_spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}

int32_t start_all_spi(void)
{
    return(pov_spi_start(spi, vspi));
}

void IRAM_ATTR kick_all(void)
{
	// please keep these in this order

	pov_kick_start(vspi);
    pov_kick_start(spi);
}

void app_main()
{
    esp_err_t ret;
    spi_bus_config_t buscfg={};
    spi_device_interface_config_t devcfg={};

    devcfg.clock_speed_hz=20000000;               //Clock out at 20 MHz
    devcfg.mode=0;                                //SPI mode 0
    devcfg.spics_io_num=-1;               //CS pin
    devcfg.queue_size=10;                          //We want to be able to queue 7 transactions at a time
    devcfg.pre_cb=ili_spi_pre_transfer_callback;  //Specify pre-transfer callback to handle D/C line
    devcfg.post_cb=ili_spi_post_transfer_callback;  //Specify post-transfer callback to handle D/C line

	buscfg.miso_io_num=-1;
	buscfg.mosi_io_num=PIN_NUM_MOSI;
	buscfg.sclk_io_num=PIN_NUM_CLK;
	buscfg.quadwp_io_num=-1;
	buscfg.quadhd_io_num=-1;

    printf("About to setup HSPI\n");

    //Initialize the SPI bus
    ret=pov_spi_bus_initialize(HSPI_HOST, &buscfg, 1);

    assert(ret==ESP_OK);
    //Attach the LCD to the SPI bus
    ret=pov_spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);

	buscfg.miso_io_num=-1;
	buscfg.mosi_io_num=PIN_NUM_VMOSI;
	buscfg.sclk_io_num=PIN_NUM_VCLK;
	buscfg.quadwp_io_num=-1;
	buscfg.quadhd_io_num=-1;

    printf("About to setup VSPI\n");

    //Initialize the SPI bus
    ret=pov_spi_bus_initialize(VSPI_HOST, &buscfg, 2);

    assert(ret==ESP_OK);
    //Attach the LCD to the SPI bus
    ret=pov_spi_bus_add_device(VSPI_HOST, &devcfg, &vspi);
    assert(ret==ESP_OK);

    //Initialize the LCD

    printf("initing\n");
    ili_init(spi);
    //Go do nice stuff.

//    gpio_app_main();

	vTaskDelay(100);

#if 0
    printf("about to kickstart vspi\n");
    pov_kick_start(vspi);
    printf("about to kickstart spi\n");
    pov_kick_start(spi);
#endif

    timer_app_main();

    for (int x = 0; ;x++)
    {
    	vTaskDelay(100);

    	// I am going to defer the start

    	// This should set a new cycle
//    	cycle_isr();

//    	printf("cycle time %d\n", cycles[x % 4]);
//    	timer_set_new_alarm(cycles[x % 4]);

    	// keep these in this order please

//    	printf("going to kick all\n");
//    	kick_all();

//        pov_kick_start(vspi);
//        pov_kick_start(spi);

//    	start_all_spi();

 //  		printf("The Hall effect sens is %d\n", hall_sensor_read());

//    	_line = (_line + 1) % 100;
    }
}


