/*
 * pov_spi.c
 *
 *  Created on: Mar 5, 2017
 *      Author: mishafarms
 */

// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
Architecture:

We can initialize a SPI driver, but we don't talk to the SPI driver itself, we address a device. A device essentially
is a combination of SPI port and CS pin, plus some information about the specifics of communication to the device
(timing, command/address length etc)

The essence of the interface to a device is a set of queues; one per device. The idea is that to send something to a SPI
device, you allocate a transaction descriptor. It contains some information about the transfer like the lenghth, address,
command etc, plus pointers to transmit and receive buffer. The address of this block gets pushed into the transmit queue.
The SPI driver does its magic, and sends and retrieves the data eventually. The data gets written to the receive buffers,
if needed the transaction descriptor is modified to indicate returned parameters and the entire thing goes into the return
queue, where whatever software initiated the transaction can retrieve it.

The entire thing is run from the SPI interrupt handler. If SPI is done transmitting/receiving but nothing is in the queue,
it will not clear the SPI interrupt but just disable it. This way, when a new thing is sent, pushing the packet into the send
queue and re-enabling the interrupt will trigger the interrupt again, which can then take care of the sending.
*/


#include "sdkconfig.h"

#include <string.h>
#include "pov_spi.h"
#include "soc/gpio_sig_map.h"
#include "soc/spi_reg.h"
#include "soc/dport_reg.h"
#include "soc/spi_struct.h"
#include "soc/rtc_cntl_reg.h"
#include "rom/ets_sys.h"
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_intr.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/xtensa_api.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "soc/soc.h"
#include "soc/dport_reg.h"
#include "soc/uart_struct.h"
#include "rom/lldesc.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "esp_heap_alloc_caps.h"

typedef struct spi_device_t spi_device_t;

#define NO_CS 3     //Number of CS pins per SPI host

extern void db_on(void);
extern void db_off(void);
extern void db_prt(int);
extern void db_prtn(int, void *, void *);

#define IS_FIRST_SPI(sp) ((sp)->hw == &SPI2)

typedef struct {
    spi_device_t *device[NO_CS];
    intr_handle_t intr;
    spi_dev_t *hw;
    spi_transaction_t *cur_trans;
    int cur_cs;
    lldesc_t dmadesc_tx[POV_SPI_MAX_DMA_CHAIN];
    bool no_gpio_matrix;
    bool inited;
} spi_host_t;

struct spi_device_t {
    QueueHandle_t trans_queue;
    QueueHandle_t ret_queue;
    spi_device_interface_config_t cfg;
    spi_host_t *host;
};

static spi_host_t *spihost[3];

static const char *SPI_TAG = "povspi_master";
#define SPI_CHECK(a, str, ret_val) \
    if (!(a)) { \
        ESP_LOGE(SPI_TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str); \
        return (ret_val); \
    }

static uint32_t two_off_leds[2] = {0x000000ff, 0x000000ff};
static uint32_t zeros[2] = {0, 0};

uint32_t leds[NUM_POV_LINES * NUM_LEDS_PER_LINE] = {};


int lineNum = 0;

/*
 Stores a bunch of per-spi-peripheral data.
*/
typedef struct {
    const uint8_t spiclk_out;       //GPIO mux output signals
    const uint8_t spid_out;
    const uint8_t spiq_out;
    const uint8_t spiwp_out;
    const uint8_t spihd_out;
    const uint8_t spid_in;          //GPIO mux input signals
    const uint8_t spiq_in;
    const uint8_t spiwp_in;
    const uint8_t spihd_in;
    const uint8_t spics_out[3];     // /CS GPIO output mux signals
    const uint8_t spiclk_native;    //IO pins of IO_MUX muxed signals
    const uint8_t spid_native;
    const uint8_t spiq_native;
    const uint8_t spiwp_native;
    const uint8_t spihd_native;
    const uint8_t spics0_native;
    const uint8_t irq;              //irq source for interrupt mux
    const uint8_t irq_dma;          //dma irq source for interrupt mux
    const periph_module_t module;   //peripheral module, for enabling clock etc
    spi_dev_t *hw;              //Pointer to the hardware registers
} spi_signal_conn_t;

/*
 Bunch of constants for every SPI peripheral: GPIO signals, irqs, hw addr of registers etc
*/
static const spi_signal_conn_t io_signal[3]={
    {
        .spiclk_out=SPICLK_OUT_IDX,
        .spid_out=SPID_OUT_IDX,
        .spiq_out=SPIQ_OUT_IDX,
        .spiwp_out=SPIWP_OUT_IDX,
        .spihd_out=SPIHD_OUT_IDX,
        .spid_in=SPID_IN_IDX,
        .spiq_in=SPIQ_IN_IDX,
        .spiwp_in=SPIWP_IN_IDX,
        .spihd_in=SPIHD_IN_IDX,
        .spics_out={SPICS0_OUT_IDX, SPICS1_OUT_IDX, SPICS2_OUT_IDX},
        .spiclk_native=6,
        .spid_native=8,
        .spiq_native=7,
        .spiwp_native=10,
        .spihd_native=9,
        .spics0_native=11,
        .irq=ETS_SPI1_INTR_SOURCE,
        .irq_dma=ETS_SPI1_DMA_INTR_SOURCE,
        .module=PERIPH_SPI_MODULE,
        .hw=&SPI1
    }, {
        .spiclk_out=HSPICLK_OUT_IDX,
        .spid_out=HSPID_OUT_IDX,
        .spiq_out=HSPIQ_OUT_IDX,
        .spiwp_out=HSPIWP_OUT_IDX,
        .spihd_out=HSPIHD_OUT_IDX,
        .spid_in=HSPID_IN_IDX,
        .spiq_in=HSPIQ_IN_IDX,
        .spiwp_in=HSPIWP_IN_IDX,
        .spihd_in=HSPIHD_IN_IDX,
        .spics_out={HSPICS0_OUT_IDX, HSPICS1_OUT_IDX, HSPICS2_OUT_IDX},
        .spiclk_native=14,
        .spid_native=13,
        .spiq_native=12,
        .spiwp_native=2,
        .spihd_native=4,
        .spics0_native=15,
        .irq=ETS_SPI2_INTR_SOURCE,
        .irq_dma=ETS_SPI2_DMA_INTR_SOURCE,
        .module=PERIPH_HSPI_MODULE,
        .hw=&SPI2
    }, {
        .spiclk_out=VSPICLK_OUT_IDX,
        .spid_out=VSPID_OUT_IDX,
        .spiq_out=VSPIQ_OUT_IDX,
        .spiwp_out=VSPIWP_OUT_IDX,
        .spihd_out=VSPIHD_OUT_IDX,
        .spid_in=VSPID_IN_IDX,
        .spiq_in=VSPIQ_IN_IDX,
        .spiwp_in=VSPIWP_IN_IDX,
        .spihd_in=VSPIHD_IN_IDX,
        .spics_out={VSPICS0_OUT_IDX, VSPICS1_OUT_IDX, VSPICS2_OUT_IDX},
        .spiclk_native=18,
        .spid_native=23,
        .spiq_native=19,
        .spiwp_native=22,
        .spihd_native=21,
        .spics0_native=5,
        .irq=ETS_SPI3_INTR_SOURCE,
        .irq_dma=ETS_SPI3_DMA_INTR_SOURCE,
        .module=PERIPH_VSPI_MODULE,
        .hw=&SPI3
    }
};

static void pov_spi_intr(void *arg);


esp_err_t pov_spi_bus_initialize(spi_host_device_t host, spi_bus_config_t *bus_config, int dma_chan)
{
    bool native=true;
    /* ToDo: remove this when we have flash operations cooperating with this */
    SPI_CHECK(host!=SPI_HOST, "SPI1 is not supported", ESP_ERR_NOT_SUPPORTED);

    SPI_CHECK(host>=SPI_HOST && host<=VSPI_HOST, "invalid host", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host]==NULL, "host already in use", ESP_ERR_INVALID_STATE);

    SPI_CHECK(bus_config->mosi_io_num<0 || GPIO_IS_VALID_OUTPUT_GPIO(bus_config->mosi_io_num), "spid pin invalid", ESP_ERR_INVALID_ARG);
    SPI_CHECK(bus_config->sclk_io_num<0 || GPIO_IS_VALID_OUTPUT_GPIO(bus_config->sclk_io_num), "spiclk pin invalid", ESP_ERR_INVALID_ARG);
    SPI_CHECK(bus_config->miso_io_num<0 || GPIO_IS_VALID_GPIO(bus_config->miso_io_num), "spiq pin invalid", ESP_ERR_INVALID_ARG);
    SPI_CHECK(bus_config->quadwp_io_num<0 || GPIO_IS_VALID_OUTPUT_GPIO(bus_config->quadwp_io_num), "spiwp pin invalid", ESP_ERR_INVALID_ARG);
    SPI_CHECK(bus_config->quadhd_io_num<0 || GPIO_IS_VALID_OUTPUT_GPIO(bus_config->quadhd_io_num), "spihd pin invalid", ESP_ERR_INVALID_ARG);

    //The host struct contains two dma descriptors, so we need DMA'able memory for this.
    spihost[host]=pvPortMallocCaps(sizeof(spi_host_t), MALLOC_CAP_DMA);
    if (spihost[host]==NULL) return ESP_ERR_NO_MEM;
    memset(spihost[host], 0, sizeof(spi_host_t));

    //Check if the selected pins correspond to the native pins of the peripheral
    if (bus_config->mosi_io_num >= 0 && bus_config->mosi_io_num!=io_signal[host].spid_native) native=false;
    if (bus_config->miso_io_num >= 0 && bus_config->miso_io_num!=io_signal[host].spiq_native) native=false;
    if (bus_config->sclk_io_num >= 0 && bus_config->sclk_io_num!=io_signal[host].spiclk_native) native=false;
    if (bus_config->quadwp_io_num >= 0 && bus_config->quadwp_io_num!=io_signal[host].spiwp_native) native=false;
    if (bus_config->quadhd_io_num >= 0 && bus_config->quadhd_io_num!=io_signal[host].spihd_native) native=false;

    spihost[host]->no_gpio_matrix=native;
    if (native) {
        //All SPI native pin selections resolve to 1, so we put that here instead of trying to figure
        //out which FUNC_GPIOx_xSPIxx to grab; they all are defined to 1 anyway.
        if (bus_config->mosi_io_num > 0) PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[bus_config->mosi_io_num], 1);
        if (bus_config->miso_io_num > 0) PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[bus_config->miso_io_num], 1);
        if (bus_config->quadwp_io_num > 0) PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[bus_config->quadwp_io_num], 1);
        if (bus_config->quadhd_io_num > 0) PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[bus_config->quadhd_io_num], 1);
        if (bus_config->sclk_io_num > 0) PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[bus_config->sclk_io_num], 1);
    } else {
        //Use GPIO
        if (bus_config->mosi_io_num>0) {
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[bus_config->mosi_io_num], PIN_FUNC_GPIO);
            gpio_set_direction(bus_config->mosi_io_num, GPIO_MODE_OUTPUT);
            gpio_matrix_out(bus_config->mosi_io_num, io_signal[host].spid_out, false, false);
            gpio_matrix_in(bus_config->mosi_io_num, io_signal[host].spid_in, false);
        }
        if (bus_config->miso_io_num>0) {
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[bus_config->miso_io_num], PIN_FUNC_GPIO);
            gpio_set_direction(bus_config->miso_io_num, GPIO_MODE_INPUT);
            gpio_matrix_out(bus_config->miso_io_num, io_signal[host].spiq_out, false, false);
            gpio_matrix_in(bus_config->miso_io_num, io_signal[host].spiq_in, false);
        }
        if (bus_config->quadwp_io_num>0) {
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[bus_config->quadwp_io_num], PIN_FUNC_GPIO);
            gpio_set_direction(bus_config->quadwp_io_num, GPIO_MODE_OUTPUT);
            gpio_matrix_out(bus_config->quadwp_io_num, io_signal[host].spiwp_out, false, false);
            gpio_matrix_in(bus_config->quadwp_io_num, io_signal[host].spiwp_in, false);
        }
        if (bus_config->quadhd_io_num>0) {
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[bus_config->quadhd_io_num], PIN_FUNC_GPIO);
            gpio_set_direction(bus_config->quadhd_io_num, GPIO_MODE_OUTPUT);
            gpio_matrix_out(bus_config->quadhd_io_num, io_signal[host].spihd_out, false, false);
            gpio_matrix_in(bus_config->quadhd_io_num, io_signal[host].spihd_in, false);
        }
        if (bus_config->sclk_io_num>0) {
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[bus_config->sclk_io_num], PIN_FUNC_GPIO);
            gpio_set_direction(bus_config->sclk_io_num, GPIO_MODE_OUTPUT);
            gpio_matrix_out(bus_config->sclk_io_num, io_signal[host].spiclk_out, false, false);
        }
    }
    periph_module_enable(io_signal[host].module);
    esp_intr_alloc(io_signal[host].irq, ESP_INTR_FLAG_INTRDISABLED, pov_spi_intr, (void*)spihost[host], &spihost[host]->intr);
    spihost[host]->hw=io_signal[host].hw;

    //Reset DMA
    spihost[host]->hw->dma_conf.val|=SPI_OUT_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST;
    spihost[host]->hw->dma_out_link.start=0;
    spihost[host]->hw->dma_in_link.start=0;
    spihost[host]->hw->dma_conf.val&=~(SPI_OUT_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST);

    //Disable unneeded ints
    spihost[host]->hw->slave.rd_buf_done=0;
    spihost[host]->hw->slave.wr_buf_done=0;
    spihost[host]->hw->slave.rd_sta_done=0;
    spihost[host]->hw->slave.wr_sta_done=0;
    spihost[host]->hw->slave.rd_buf_inten=0;
    spihost[host]->hw->slave.wr_buf_inten=0;
    spihost[host]->hw->slave.rd_sta_inten=0;
    spihost[host]->hw->slave.wr_sta_inten=0;

    //Force a transaction done interrupt. This interrupt won't fire yet because we initialized the SPI interrupt as
    //disabled.  This way, we can just enable the SPI interrupt and the interrupt handler will kick in, handling
    //any transactions that are queued.
    spihost[host]->hw->slave.trans_inten=1;
    spihost[host]->hw->slave.trans_done=1;

    //Select DMA channel.
    SET_PERI_REG_BITS(DPORT_SPI_DMA_CHAN_SEL_REG, 3, dma_chan, (host * 2));

    return ESP_OK;
}

esp_err_t pov_spi_bus_free(spi_host_device_t host)
{
    int x;
    SPI_CHECK(host>=SPI_HOST && host<=VSPI_HOST, "invalid host", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host]!=NULL, "host not in use", ESP_ERR_INVALID_STATE);
    for (x=0; x<NO_CS; x++) {
        SPI_CHECK(spihost[host]->device[x]==NULL, "not all CSses freed", ESP_ERR_INVALID_STATE);
    }
    spihost[host]->hw->slave.trans_inten=0;
    spihost[host]->hw->slave.trans_done=0;
    esp_intr_free(spihost[host]->intr);
    periph_module_disable(io_signal[host].module);
    free(spihost[host]);
    spihost[host]=NULL;
    return ESP_OK;
}

/*
 Add a device. This allocates a CS line for the device, allocates memory for the device structure and hooks
 up the CS pin to whatever is specified.
*/
esp_err_t pov_spi_bus_add_device(spi_host_device_t host, spi_device_interface_config_t *dev_config, spi_device_handle_t *handle)
{
    int freecs;
    SPI_CHECK(host>=SPI_HOST && host<=VSPI_HOST, "invalid host", ESP_ERR_INVALID_ARG);
    SPI_CHECK(spihost[host]!=NULL, "host not initialized", ESP_ERR_INVALID_STATE);
    SPI_CHECK(dev_config->spics_io_num < 0 || GPIO_IS_VALID_OUTPUT_GPIO(dev_config->spics_io_num), "spics pin invalid", ESP_ERR_INVALID_ARG);
    SPI_CHECK(dev_config->clock_speed_hz > 0, "invalid sclk speed", ESP_ERR_INVALID_ARG);
    for (freecs=0; freecs<NO_CS; freecs++) {
        //See if this slot is free; reserve if it is by putting a dummy pointer in the slot. We use an atomic compare&swap to make this thread-safe.
        if (__sync_bool_compare_and_swap(&spihost[host]->device[freecs], NULL, (spi_device_t *)1)) break;
    }
    SPI_CHECK(freecs!=NO_CS, "no free cs pins for host", ESP_ERR_NOT_FOUND);
    //The hardware looks like it would support this, but actually setting cs_ena_pretrans when transferring in full
    //duplex mode does absolutely nothing on the ESP32.
    SPI_CHECK(dev_config->cs_ena_pretrans==0 || (dev_config->flags & SPI_DEVICE_HALFDUPLEX), "cs pretrans delay incompatible with full-duplex", ESP_ERR_INVALID_ARG);

    //Allocate memory for device
    spi_device_t *dev=malloc(sizeof(spi_device_t));
    if (dev==NULL) return ESP_ERR_NO_MEM;
    memset(dev, 0, sizeof(spi_device_t));
    spihost[host]->device[freecs]=dev;

    //Allocate queues, set defaults
    dev->trans_queue=xQueueCreate(dev_config->queue_size, sizeof(spi_transaction_t *));
    dev->ret_queue=xQueueCreate(dev_config->queue_size, sizeof(spi_transaction_t *));
    if (dev_config->duty_cycle_pos==0) dev_config->duty_cycle_pos=128;
    dev->host=spihost[host];

    //We want to save a copy of the dev config in the dev struct.
    memcpy(&dev->cfg, dev_config, sizeof(spi_device_interface_config_t));

    //Set CS pin, CS options
    if (dev_config->spics_io_num > 0) {
        if (spihost[host]->no_gpio_matrix &&dev_config->spics_io_num == io_signal[host].spics0_native && freecs==0) {
            //Again, the cs0s for all SPI peripherals map to pin mux source 1, so we use that instead of a define.
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[dev_config->spics_io_num], 1);
        } else {
            //Use GPIO matrix
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[dev_config->spics_io_num], PIN_FUNC_GPIO);
            gpio_set_direction(dev_config->spics_io_num, GPIO_MODE_OUTPUT);
            gpio_matrix_out(dev_config->spics_io_num, io_signal[host].spics_out[freecs], false, false);
        }
    }
    if (dev_config->flags&SPI_DEVICE_CLK_AS_CS) {
        spihost[host]->hw->pin.master_ck_sel |= (1<<freecs);
    } else {
        spihost[host]->hw->pin.master_ck_sel &= (1<<freecs);
    }
    if (dev_config->flags&SPI_DEVICE_POSITIVE_CS) {
        spihost[host]->hw->pin.master_cs_pol |= (1<<freecs);
    } else {
        spihost[host]->hw->pin.master_cs_pol &= (1<<freecs);
    }
    *handle=dev;
    return ESP_OK;
}

esp_err_t pov_spi_bus_remove_device(spi_device_handle_t handle)
{
    int x;
    SPI_CHECK(handle!=NULL, "invalid handle", ESP_ERR_INVALID_ARG);
    //These checks aren't exhaustive; another thread could sneak in a transaction inbetween. These are only here to
    //catch design errors and aren't meant to be triggered during normal operation.
    SPI_CHECK(uxQueueMessagesWaiting(handle->trans_queue)==0, "Have unfinished transactions", ESP_ERR_INVALID_STATE);
    SPI_CHECK(handle->host->cur_trans==0 || handle->host->device[handle->host->cur_cs]!=handle, "Have unfinished transactions", ESP_ERR_INVALID_STATE);
    SPI_CHECK(uxQueueMessagesWaiting(handle->ret_queue)==0, "Have unfinished transactions", ESP_ERR_INVALID_STATE);

    //Kill queues
    vQueueDelete(handle->trans_queue);
    vQueueDelete(handle->ret_queue);
    //Remove device from list of csses and free memory
    for (x=0; x<NO_CS; x++) {
        if (handle->host->device[x] == handle) handle->host->device[x]=NULL;
    }
    free(handle);
    return ESP_OK;
}

static int spi_freq_for_pre_n(int fapb, int pre, int n) {
    return (fapb / (pre * n));
}

static void spi_set_clock(spi_dev_t *hw, int fapb, int hz, int duty_cycle) {
    int pre, n, h, l;

    //In hw, n, h and l are 1-64, pre is 1-8K. Value written to register is one lower than used value.
    if (hz>((fapb/4)*3)) {
        //Using Fapb directly will give us the best result here.
        hw->clock.clkcnt_l=0;
        hw->clock.clkcnt_h=0;
        hw->clock.clkcnt_n=0;
        hw->clock.clkdiv_pre=0;
        hw->clock.clk_equ_sysclk=1;
    } else {
        //For best duty cycle resolution, we want n to be as close to 32 as possible, but
        //we also need a pre/n combo that gets us as close as possible to the intended freq.
        //To do this, we bruteforce n and calculate the best pre to go along with that.
        //If there's a choice between pre/n combos that give the same result, use the one
        //with the higher n.
        int bestn=-1;
        int bestpre=-1;
        int besterr=0;
        int errval;
        for (n=1; n<=64; n++) {
            //Effectively, this does pre=round((fapb/n)/hz).
            pre=((fapb/n)+(hz/2))/hz;
            if (pre<=0) pre=1;
            if (pre>8192) pre=8192;
            errval=abs(spi_freq_for_pre_n(fapb, pre, n)-hz);
            if (bestn==-1 || errval<=besterr) {
                besterr=errval;
                bestn=n;
                bestpre=pre;
            }
        }

        n=bestn;
        pre=bestpre;
        l=n;
        //This effectively does round((duty_cycle*n)/256)
        h=(duty_cycle*n+127)/256;
        if (h<=0) h=1;

        hw->clock.clk_equ_sysclk=0;
        hw->clock.clkcnt_n=n-1;
        hw->clock.clkdiv_pre=pre-1;
        hw->clock.clkcnt_h=h-1;
        hw->clock.clkcnt_l=l-1;
    }
}


//If a transaction is smaller than or equal to of bits, we do not use DMA; instead, we directly copy/paste
//bits from/to the work registers. Keep between 32 and (8*32) please.
#define THRESH_DMA_TRANS (8*32)

//This is run in interrupt context and apart from initialization and destruction, this is the only code
//touching the host (=spihost[x]) variable. The rest of the data arrives in queues. That is why there are
//no muxes in this code.
static void IRAM_ATTR pov_spi_intr(void *arg)
{
    int i;
    spi_host_t *host=(spi_host_t*)arg;
    int total_len = 0;
    int numLedStrands = 0;

	//Ignore all but the trans_done int.
    if (!host->hw->slave.trans_done)
    {
    	// This is really bad and shouldn't be ignored (I am not turning off the interrupt enable like they did)
    	return;
    }

    // we can get here because 1 of 2 DMA channels is finished.

    // stop the pain, clear the int bit

    host->hw->slave.trans_done=0;

    //because we are dealing with LEDS I am going to totally ignore the rx part of this

    // we do a slightly different setup depending on which DMA channel this is. I am using
    //Call post-transaction callback, if any
    if (host->device[host->cur_cs]->cfg.post_cb)
    {
//    	host->device[host->cur_cs]->cfg.post_cb(IS_FIRST_SPI(host) ? 25 : 26);
    }

    // if the lineNum is >= MAX_LINES then do nothing here

    if (lineNum > NUM_POV_LINES)
    {
    	return;
    }

    /* so we are going to setup the DMA for the next line of LEDS, it will be triggered by a timer interrupt to occur
     * at a synchronized time.
     */

    {
        //We have a transaction. Send it.
        spi_device_t *dev=host->device[0];
        //We should be done with the transmission.
//        assert(host->hw->cmd.usr == 0);

        //Reconfigure according to device settings, but only if we change CSses.
        if (!host->inited)
        {
        	int x;

            //Assumes a hardcoded 80MHz Fapb for now. ToDo: figure out something better once we have
            //clock scaling working.
            int apbclk=APB_CLK_FREQ;
            spi_set_clock(host->hw, apbclk, dev->cfg.clock_speed_hz, dev->cfg.duty_cycle_pos);
            //Configure bit order
            host->hw->ctrl.rd_bit_order=(dev->cfg.flags & SPI_DEVICE_RXBIT_LSBFIRST)?1:0;
            host->hw->ctrl.wr_bit_order=(dev->cfg.flags & SPI_DEVICE_TXBIT_LSBFIRST)?1:0;

            //Configure polarity
            //SPI iface needs to be configured for a delay unless it is not routed through GPIO and clock is >=apb/2
            int nodelay=(host->no_gpio_matrix && dev->cfg.clock_speed_hz >= (apbclk/2));
            if (dev->cfg.mode==0) {
                host->hw->pin.ck_idle_edge=0;
                host->hw->user.ck_out_edge=0;
                host->hw->ctrl2.miso_delay_mode=nodelay?0:2;
            } else if (dev->cfg.mode==1) {
                host->hw->pin.ck_idle_edge=0;
                host->hw->user.ck_out_edge=1;
                host->hw->ctrl2.miso_delay_mode=nodelay?0:1;
            } else if (dev->cfg.mode==2) {
                host->hw->pin.ck_idle_edge=1;
                host->hw->user.ck_out_edge=1;
                host->hw->ctrl2.miso_delay_mode=nodelay?0:1;
            } else if (dev->cfg.mode==3) {
                host->hw->pin.ck_idle_edge=1;
                host->hw->user.ck_out_edge=0;
                host->hw->ctrl2.miso_delay_mode=nodelay?0:2;
            }

            //Configure bit sizes, load addr and command
            host->hw->user.usr_dummy=(dev->cfg.dummy_bits)?1:0;
            host->hw->user.usr_addr=(dev->cfg.address_bits)?1:0;
            host->hw->user.usr_command=(dev->cfg.command_bits)?1:0;
            host->hw->user1.usr_addr_bitlen=dev->cfg.address_bits-1;
            host->hw->user1.usr_dummy_cyclelen=dev->cfg.dummy_bits-1;
            host->hw->user2.usr_command_bitlen=dev->cfg.command_bits-1;
            //Configure misc stuff
            host->hw->user.doutdin=(dev->cfg.flags & SPI_DEVICE_HALFDUPLEX)?0:1;
            host->hw->user.sio=(dev->cfg.flags & SPI_DEVICE_3WIRE)?1:0;

            host->hw->ctrl2.setup_time=dev->cfg.cs_ena_pretrans-1;
            host->hw->user.cs_setup=dev->cfg.cs_ena_pretrans?1:0;
            host->hw->ctrl2.hold_time=dev->cfg.cs_ena_posttrans-1;
            host->hw->user.cs_hold=(dev->cfg.cs_ena_posttrans)?1:0;

            //Configure CS pin
            host->hw->pin.cs0_dis=1;
            host->hw->pin.cs1_dis=1;
            host->hw->pin.cs2_dis=1;

            // the DMA chain will look almost always the same. So set it up now

        	host->dmadesc_tx[0].length = 4;
        	host->dmadesc_tx[0].size = 4 ;
        	host->dmadesc_tx[0].buf = (uint8_t*) &zeros;
        	host->dmadesc_tx[0].eof = 1;
        	host->dmadesc_tx[0].sosf = 0;
        	host->dmadesc_tx[0].empty = (uint32_t)(&host->dmadesc_tx[1]);

        	if (IS_FIRST_SPI(host))
        	{
        		numLedStrands = 3;
        	}
        	else
        	{
        		numLedStrands = 2;
        	}

        	for (x = 1 ; x < (numLedStrands + 1) ; x++)
        	{
            	host->dmadesc_tx[x].length = NUM_LEDS_PER_LINE * 4;
            	host->dmadesc_tx[x].size = NUM_LEDS_PER_LINE * 4;
            	// we can't really set the buffer as that is what will change
            	host->dmadesc_tx[x].eof = 1;
            	host->dmadesc_tx[x].sosf = 0;
            	host->dmadesc_tx[x].empty = (uint32_t)(&host->dmadesc_tx[x + 1]);
        	}

        	host->dmadesc_tx[x].length = 4;
        	host->dmadesc_tx[x].size = 4 ;
        	host->dmadesc_tx[x].buf = (uint8_t*) &two_off_leds;
        	host->dmadesc_tx[x].eof = 1;
        	host->dmadesc_tx[x].sosf = 0;
        	host->dmadesc_tx[x].empty = (uint32_t) NULL;

        	host->inited = true;
        }

        //Reset DMA
        host->hw->dma_conf.val |= SPI_OUT_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST;
        host->hw->dma_out_link.start=0;
        host->hw->dma_in_link.start=0;
        host->hw->dma_conf.val &= ~(SPI_OUT_RST|SPI_AHBM_RST|SPI_AHBM_FIFO_RST);
        //QIO/DIO
        host->hw->ctrl.val &= ~(SPI_FREAD_DUAL|SPI_FREAD_QUAD|SPI_FREAD_DIO|SPI_FREAD_QIO);
        host->hw->user.val &= ~(SPI_FWRITE_DUAL|SPI_FWRITE_QUAD|SPI_FWRITE_DIO|SPI_FWRITE_QIO);

        // we are not using RX, so disable it here.

        host->hw->user.usr_miso=0;

        // we need to figure out which host this is and then we can load the proper info

        host->hw->user.usr_mosi_highpart=0;

        if (IS_FIRST_SPI(host))
        {
        	int x;
        	// we need to figure out which line we are on and then grab the correct lines to display

        	host->dmadesc_tx[0].owner = 1;

        	for (x = 1 ; x < 4 ; x++)
        	{
        		int line = (((x - 1) * POV_LINE_OFFSET) + lineNum) % NUM_POV_LINES;

//        		db_prt(line);

        		host->dmadesc_tx[x].buf = (uint8_t *) (leds + (NUM_LEDS_PER_LINE * line));
            	host->dmadesc_tx[x].owner = 1;
        	}

        	// set the owner bit of the last piece of the chain

        	host->dmadesc_tx[x].owner = 1;

        	total_len = 1600; // start 1 + 16 * 3 + stop 1 50 * 32 = 1600

        	// increment once per cycle

           ++lineNum;  // I am not going to do anything here. If the lineNum is >= NUM_POV_LINES, then do nothing

//        	lineNum = 0;

        }
        else
        {
        	int x;
        	// we need to figure out which line we are on and then grab the correct lines to display

        	host->dmadesc_tx[0].owner = 1;

        	for (x = 1 ; x < 3 ; x++)
        	{
        		int line = (((x + 2) * POV_LINE_OFFSET) + lineNum) % NUM_POV_LINES;

//        		db_prt(line);

        		host->dmadesc_tx[x].buf = (uint8_t *) (leds + (NUM_LEDS_PER_LINE * line));
            	host->dmadesc_tx[x].owner = 1;
        	}

        	// set the owner bit of the last piece of the chain

        	host->dmadesc_tx[x].owner = 1;

        	total_len = 1088; // start 1 + 16 * 2 + stop 1 34 * 32 = 1088
        }

        host->hw->dma_out_link.addr = (int)(&host->dmadesc_tx[0]) & 0xFFFFF;
        host->hw->dma_out_link.start = 1;

        host->hw->mosi_dlen.usr_mosi_dbitlen = total_len - 1;
        host->hw->miso_dlen.usr_miso_dbitlen = 0;
        host->hw->user2.usr_command_value = 0;

        host->hw->addr = 0;

        host->hw->user.usr_mosi = 1;
        host->hw->user.usr_miso = 0;
    }
}

int32_t IRAM_ATTR pov_spi_start(spi_device_handle_t handle, spi_device_handle_t handle2)
{
    spi_device_t *dev=handle->host->device[handle->host->cur_cs];

//    SPI_CHECK(handle!=NULL, "invalid dev handle", ESP_ERR_INVALID_ARG);
//    SPI_CHECK(handle2!=NULL, "invalid dev handle2", ESP_ERR_INVALID_ARG);


	//Call pre-transmission callback, if any
	if (dev->cfg.pre_cb)
	{
//		dev->cfg.pre_cb(25);
//		dev->cfg.pre_cb(26);
	}

    //Kick off transfer
    handle->host->hw->cmd.usr=1;
    handle2->host->hw->cmd.usr=1;

    return lineNum;
}

esp_err_t pov_kick_start(spi_device_handle_t handle)
{
//	if (IS_FIRST_SPI(handle->host))
	{
		lineNum = 0;
	}
	handle->host->hw->slave.trans_done = 1;
    esp_intr_enable(handle->host->intr);
    return ESP_OK;
}

esp_err_t pov_spi_device_queue_trans(spi_device_handle_t handle, spi_transaction_t *trans_desc,  TickType_t ticks_to_wait)
{
    BaseType_t r;
    SPI_CHECK(handle!=NULL, "invalid dev handle", ESP_ERR_INVALID_ARG);
    SPI_CHECK((trans_desc->flags & SPI_TRANS_USE_RXDATA)==0 ||trans_desc->length <= 32, "rxdata transfer > 32bytes", ESP_ERR_INVALID_ARG);
    SPI_CHECK((trans_desc->flags & SPI_TRANS_USE_TXDATA)==0 ||trans_desc->length <= 32, "txdata transfer > 32bytes", ESP_ERR_INVALID_ARG);
    SPI_CHECK(!((trans_desc->flags & (SPI_TRANS_MODE_DIO|SPI_TRANS_MODE_QIO)) && (handle->cfg.flags & SPI_DEVICE_3WIRE)), "incompatible iface params", ESP_ERR_INVALID_ARG);
    SPI_CHECK(!((trans_desc->flags & (SPI_TRANS_MODE_DIO|SPI_TRANS_MODE_QIO)) && (!(handle->cfg.flags & SPI_DEVICE_HALFDUPLEX))), "incompatible iface params", ESP_ERR_INVALID_ARG);
    r=xQueueSend(handle->trans_queue, (void*)&trans_desc, ticks_to_wait);
    if (!r) return ESP_ERR_TIMEOUT;
    esp_intr_enable(handle->host->intr);
    return ESP_OK;
}

esp_err_t pov_spi_device_get_trans_result(spi_device_handle_t handle, spi_transaction_t **trans_desc, TickType_t ticks_to_wait)
{
    BaseType_t r;
    SPI_CHECK(handle!=NULL, "invalid dev handle", ESP_ERR_INVALID_ARG);
    r=xQueueReceive(handle->ret_queue, (void*)trans_desc, ticks_to_wait);
    if (!r) return ESP_ERR_TIMEOUT;
    return ESP_OK;
}

//Porcelain to do one blocking transmission.
esp_err_t pov_spi_device_transmit(spi_device_handle_t handle, spi_transaction_t *trans_desc)
{
    esp_err_t ret;
    spi_transaction_t *ret_trans;
    //ToDo: check if any spi transfers in flight
    ret=pov_spi_device_queue_trans(handle, trans_desc, portMAX_DELAY);
    if (ret!=ESP_OK) return ret;
    ret=pov_spi_device_get_trans_result(handle, &ret_trans, portMAX_DELAY);
    if (ret!=ESP_OK) return ret;
    assert(ret_trans==trans_desc);
    return ESP_OK;
}

