/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016-2021 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <string.h>

#include "py/runtime.h"
#include "py/mphal.h"
#include "shared/runtime/mpirq.h"
#include "modmachine.h"
#include "extmod/virtpin.h"

#include "hardware/irq.h"
#include "hardware/regs/intctrl.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/padsbank0.h"

#include "hardware/structs/dma.h"
#include "hardware/dma.h"
#include "pio_adc_read.pio.h"
#include "pio_aps6408l_write.pio.h"

#define ADC_BASE_PIN 2
#define ADC_BUS_WIDTH 8
#define ADC_PIO_FREQ 100000000
#define ADC_SM 0
#define ADC_BLOCK_SIZE 1024

#define APS6408L_WRITE_SM 1
#define APS6408L_BYTE_SIZE (uint32_t)16*1024*1024//16Mbytes
#define APS6408L_PIO_FREQ 125000000
#define APS6408L_BASE_PIN 18
#define APS6408L_CLK_PIN 26
#define APS6408L_DQS_PIN 27
#define APS6408L_CE_PIN 28
#define APS6408L_RST_PIN 29
#define APS6408L_BUS_WIDTH 8
#define APS6408L_BUS_MASK (uint32_t)0X000000FF<<APS6408L_BASE_PIN

#define APS6408L_DISORDER_PINS(ordered) (uint32_t)(((ordered&0x01)<<4)|((ordered&0x02)<<2)|((ordered&0x04)<<4)|((ordered&0x08)<<2)|((ordered&0x10)<<3)|((ordered&0xE0)>>5))
#define APS6408L_ORDER_PINS(disordered) (uint32_t)(((disordered&0x07)<<5)|((disordered&0x08)>>2)|((disordered&0x10)>>4)|((disordered&0x20)>>2)|((disordered&0x40)>>4)|((disordered&0x80)>>3))

#define APS6408L_REGISTER_MODE_0        0
#define APS6408L_REGISTER_MODE_1        1
#define APS6408L_REGISTER_MODE_2        2
#define APS6408L_REGISTER_MODE_3        3
#define APS6408L_REGISTER_MODE_4        4
#define APS6408L_REGISTER_MODE_8        8

#define APS6408L_HYBRID_MODE            (1<<2)
#define APS6408L_16_WRAP                0X00
#define APS6408L_32_WRAP                0X01
#define APS6408L_64_WRAP                0X02
#define APS6408L_1024_WRAP              0X07

#define APS6408L_COMMAND_SYNC_READ          0X00
#define APS6408L_COMMAND_SYNC_WRITE         0X80
#define APS6408L_COMMAND_SYNC_READ_BURST    0X20
#define APS6408L_COMMAND_SYNC_WRITE_BURST   0XA0
#define APS6408L_COMMAND_REGISTER_READ      0X40
#define APS6408L_COMMAND_REGISTER_WRITE     0XC0
#define APS6408L_COMMAND_RESET              0XFF

volatile uint8_t adc_buffer_a[ADC_BLOCK_SIZE];
volatile uint8_t adc_buffer_b[ADC_BLOCK_SIZE];
volatile uint8_t *adc_ptr_a;
volatile uint8_t *adc_ptr_b;
int dma_chan_a;//channel buffer a
int dma_chan_control_a;//channel a control
int dma_chan_b;//channel buffer b
int dma_chan_control_b;//channel b control
volatile uint32_t mem_address;


STATIC void dma_handler(void) {
    uint32_t intr = dma_hw->ints0;
    if (intr & (0x00000001<<dma_chan_a)) {
        dma_hw->ints0&=~(0x00000001<<dma_chan_a);
        mem_address=(mem_address+1024)&(APS6408L_BYTE_SIZE-1);
//        machine_pin_irq_obj_t *irq = MP_STATE_PORT(machine_pin_irq_obj[30]);
  //      mp_irq_handler(&irq->base);
    }
    if (intr & (0x00000001<<dma_chan_b)) {
        dma_hw->ints0&=~(0x00000001<<dma_chan_b);
        mem_address=(mem_address+1024)&(APS6408L_BYTE_SIZE-1);
      //  machine_pin_irq_obj_t *irq = MP_STATE_PORT(machine_pin_irq_obj[31]);
    //    mp_irq_handler(&irq->base);
    }
}


void machine_oscilloscope_init(void){
    //init adc read pio
    uint offset = pio_add_program(pio0, &pio_adc_read_program);
    pio_adc_read_program_init(pio0, ADC_SM, offset, ADC_BASE_PIN, ADC_BUS_WIDTH, ADC_PIO_FREQ);

    //initialize data control pointers
    adc_ptr_a = adc_buffer_a;
    adc_ptr_b = adc_buffer_b;
    mem_address=0;

    // Claim 4 dma channels to dos the chain
    dma_chan_a = dma_claim_unused_channel(true);
    dma_chan_control_a = dma_claim_unused_channel(true);
    dma_chan_b = dma_claim_unused_channel(true);
    dma_chan_control_b = dma_claim_unused_channel(true);

    // Tell the chan_a and chan_b to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(dma_chan_a, true);
    dma_channel_set_irq0_enabled(dma_chan_control_a, false);
    dma_channel_set_irq0_enabled(dma_chan_b, true);
    dma_channel_set_irq0_enabled(dma_chan_control_b, false);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    //set the whole dma chain
    //  |-------|
    //  |       V
    //  |   dma_chan_a
    //  |   (fill buffer a)
    //  |       |---->DMA_IRQ_0---->Address++ Set@bufferB
    //  |       |
    //  |   Multitrigger--->dma_memory_a
    //  |       |                    
    //  |       V
    //  |   dma_chan_control_a
    //  |   (re-set dma_chan_a)
    //  |       |
    //  |       V
    //  |   dma_chan_b
    //  |   (fill buffer b)            
    //  |       |---->DMA_IRQ_0---->Address++ Set@bufferA
    //  |       |
    //  |   Multitrigger--->dma_memory_b
    //  |       |                    
    //  |       V
    //  |   dma_chan_control_b         
    //  |   (re-set dma_chan_b)        
    //  |-------|                      
    //      
    dma_channel_config c;

    c= dma_channel_get_default_config(dma_chan_control_a);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_irq_quiet(&c ,false);
    channel_config_set_chain_to(&c, dma_chan_b);
    channel_config_set_high_priority( &c, true);
    dma_channel_configure(
        dma_chan_control_a,
        &c,
        &dma_hw->ch[dma_chan_a].write_addr,
        &adc_ptr_a,   // Pointer from adc_buffer_a
        1,           // Write the same value many times
        false                       // Don't start yet
    );

    c= dma_channel_get_default_config(dma_chan_b);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio0, ADC_SM, false));
    channel_config_set_irq_quiet(&c ,false);
    channel_config_set_chain_to(&c, dma_chan_control_b);
    channel_config_set_high_priority( &c, true);
    dma_channel_configure(
        dma_chan_b,
        &c,
        adc_buffer_b,               // Write address 
        &pio0_hw->rxf[ADC_SM],
        ADC_BLOCK_SIZE/4,           // Write the same value many times
        false                       // Don't start yet
    );

    c= dma_channel_get_default_config(dma_chan_control_b);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_irq_quiet(&c ,false);
    channel_config_set_chain_to(&c, dma_chan_a);
    channel_config_set_high_priority( &c, true);
    dma_channel_configure(
        dma_chan_control_b,
        &c,
        &dma_hw->ch[dma_chan_b].write_addr,
        &adc_ptr_b,   // Pointer from adc_buffer_b
        1,           // Write the same value many times
        false                       // Don't start yet
    );

    c= dma_channel_get_default_config(dma_chan_a);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio0, ADC_SM, false));
    channel_config_set_irq_quiet(&c ,false);
    channel_config_set_chain_to(&c, dma_chan_control_a);
    channel_config_set_high_priority( &c, true);
    dma_channel_configure(
        dma_chan_a,
        &c,
        adc_buffer_a,               // Write address 
        &pio0_hw->rxf[ADC_SM],
        ADC_BLOCK_SIZE/4,           // Write the same value many times
        true                       // trigger now
    );

    //Memory setting
    for (int gpio = APS6408L_BASE_PIN; gpio < APS6408L_BASE_PIN + APS6408L_BUS_WIDTH; gpio++) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_OUT);
    }
    gpio_init(APS6408L_CLK_PIN);
    gpio_set_dir(APS6408L_CLK_PIN, GPIO_OUT);
    gpio_init(APS6408L_DQS_PIN);
    gpio_set_dir(APS6408L_DQS_PIN, GPIO_IN);
    gpio_init(APS6408L_CE_PIN);
    gpio_set_dir(APS6408L_CE_PIN, GPIO_OUT);
    gpio_init(APS6408L_RST_PIN);
    gpio_set_dir(APS6408L_RST_PIN, GPIO_OUT);

    gpio_put(APS6408L_CLK_PIN,0);
    //gpio_put(APS6408L_DQS_PIN,0);
    gpio_put(APS6408L_CE_PIN,1);
    sleep_us(200);
    gpio_put(APS6408L_RST_PIN,0);
    sleep_us(10);
    gpio_put(APS6408L_RST_PIN,1);
    sleep_us(10);
    
    uint8_t command[8];
    command[0] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_REGISTER_WRITE);
    command[1] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_REGISTER_WRITE);
    command[2] = APS6408L_DISORDER_PINS((APS6408L_REGISTER_MODE_8&0xFF000000)>>24);
    command[3] = APS6408L_DISORDER_PINS((APS6408L_REGISTER_MODE_8&0x00FF0000)>>16);
    command[4] = APS6408L_DISORDER_PINS((APS6408L_REGISTER_MODE_8&0x0000FF00)>>8);
    command[5] = APS6408L_DISORDER_PINS((APS6408L_REGISTER_MODE_8&0x000000FF));
    command[6] = APS6408L_DISORDER_PINS((APS6408L_1024_WRAP&0x000000FF));//1kbyte wrap
    command[7] = APS6408L_DISORDER_PINS((APS6408L_1024_WRAP&0x000000FF));

    gpio_put(APS6408L_CE_PIN,0);
    for(uint8_t x=0;x<8;x+=2){
        gpio_put_masked(APS6408L_BUS_MASK,(uint32_t)command[x]<<APS6408L_BASE_PIN);
        gpio_put(APS6408L_CLK_PIN,1);
        gpio_put_masked(APS6408L_BUS_MASK,(uint32_t)command[x+1]<<APS6408L_BASE_PIN);
        gpio_put(APS6408L_CLK_PIN,0);
    }
    gpio_put(APS6408L_CE_PIN,1);

    offset = pio_add_program(pio0, &pio_aps6408l_write_program);
    pio_aps6408l_write_program_init(pio0, APS6408L_WRITE_SM, offset, APS6408L_BASE_PIN, APS6408L_BUS_WIDTH, APS6408L_CLK_PIN, APS6408L_PIO_FREQ);
 
    gpio_set_dir(APS6408L_DQS_PIN, GPIO_OUT);
    gpio_put(APS6408L_DQS_PIN,0);
    gpio_put(APS6408L_CE_PIN,0);

    uint8_t buff_test[1024+14] __attribute__((aligned(sizeof(uint32_t *))));
    for(uint16_t x=0;x<1024;x++) buff_test[x+14]=x%256;
    buff_test[0] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_SYNC_WRITE_BURST);
    buff_test[1] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_SYNC_WRITE_BURST);
    buff_test[2] = APS6408L_DISORDER_PINS((1024&0xFF000000)>>24);
    buff_test[3] = APS6408L_DISORDER_PINS((1024&0x00FF0000)>>16);
    buff_test[4] = APS6408L_DISORDER_PINS((1024&0x0000FF00)>>8);
    buff_test[5] = APS6408L_DISORDER_PINS((1024&0x000000FF));

    //uint16_t *ptr32=(uint16_t *)&buff_test[0]; 
    //for(uint16_t x=0;x<519;x++) pio_sm_put_blocking(pio0, APS6408L_WRITE_SM, *ptr32++);

    uint dma_mem;
    dma_mem = dma_claim_unused_channel(true);
    c= dma_channel_get_default_config(dma_mem);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio0, APS6408L_WRITE_SM, true));
    channel_config_set_irq_quiet(&c ,false);
    //channel_config_set_chain_to(&c, dma_chan_control_b);
    //channel_config_set_high_priority( &c, true);
    dma_channel_configure(
        dma_mem,
        &c,
        &pio0_hw->txf[APS6408L_WRITE_SM],              //write
        buff_test,               //read 
        1038/2,                     // Write the same value many times
        true                       // start
    );
    while(pio_sm_is_tx_fifo_empty(pio0, APS6408L_WRITE_SM)!=true);

    gpio_put(APS6408L_CE_PIN,1);
    gpio_set_dir(APS6408L_DQS_PIN, GPIO_IN);


    gpio_set_dir(APS6408L_DQS_PIN, GPIO_OUT);
    gpio_put(APS6408L_DQS_PIN,0);
    gpio_put(APS6408L_CE_PIN,0);

    buff_test[2] = APS6408L_DISORDER_PINS((2048&0xFF000000)>>24);
    buff_test[3] = APS6408L_DISORDER_PINS((2048&0x00FF0000)>>16);
    buff_test[4] = APS6408L_DISORDER_PINS((2048&0x0000FF00)>>8);
    buff_test[5] = APS6408L_DISORDER_PINS((2048&0x000000FF));

    dma_channel_configure(
        dma_mem,
        &c,
        &pio0_hw->txf[APS6408L_WRITE_SM],              //write
        buff_test,               //read 
        1038/2,                     // Write the same value many times
        true                       // start
    );
    while(pio_sm_is_tx_fifo_empty(pio0, APS6408L_WRITE_SM)!=true);

    gpio_put(APS6408L_CE_PIN,1);
    gpio_set_dir(APS6408L_DQS_PIN, GPIO_IN);

 }
