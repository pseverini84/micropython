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
/*
    INFORMATION ABOUT DATA.BIN FILE
    --------------------------------
    FIRST BLOCK RESERVED FOR INFORMATION
    SECOND BLOCK RESERVED FOR EXT TRIGGER INFORMATION (ADDRESSES WHERE EXT TRIGGER WAS RAISED)
    THIRD BLOCK IS NO USED YET
    FOURTH BLOCK ONWARD USED FOR MEASURE

    BLOCK SIZE IS EQUAL TO APS6408L_BLOCK_SIZE
*/

#include <stdio.h>
#include <string.h>

//#include "py/runtime.h"
#include "py/mphal.h"
#include "shared/runtime/mpirq.h"
#include "modmachine.h"
#include "extmod/virtpin.h"

#include "extmod/vfs.h"

#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/regs/intctrl.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/padsbank0.h"

#include "hardware/structs/dma.h"
#include "hardware/dma.h"
#include "pio_adc_read.pio.h"

#include "aps6408l.h"
#include "ram_fat_mock.h"

// Include MicroPython API.
#include "py/runtime.h"

///////////////////////////////////////////////////
//HARDWARE RELATED DEFINES
#define ADC_EXT_TRIGGER_PIN 0
#define ADC_VERTICAL_RESOLUTION_PIN 1//
#define ADC_TRIGGER_PIN 10//analog trigger pin

#define ADC_BASE_PIN 2 
#define ADC_BUS_WIDTH 8 
#define ADC_BUS_MASK 0x000000FF<<ADC_BASE_PIN
#define ADC_CLK_PIN 11
#define ADC_PIO_FREQ 200000000
#define ADC_SM 0
#define ADC_BLOCK_SIZE 1024//APS6408L_BLOCK_SIZE
///////////////////////////////////////////////////
//LOGIC RELATED DEFINES
#define ADC_VIRTUAL_GROUND_LEVEL 0x80 //128
#define ADC_MIN_SAMPLE_RATE 20 //20MS/S
#define ADC_MAX_SAMPLE_RATE 100 //20MS/S
///////////////////////////////////////////////////
//RAM ORGANIZATION RELATED DEFINES
#define ADC_RAM_INFORMATION_OFFSET              (uint32_t)(APS6408L_BLOCK_SIZE*0)
#define ADC_RAM_EXT_TRIGGER_OFFSET              (uint32_t)(APS6408L_BLOCK_SIZE*1)
#define ADC_RAM_MEASUREMENT_INITIAL_OFFSET      (uint32_t)(APS6408L_BLOCK_SIZE*2)
#define ADC_RAM_INFORMATION_ADDRESS             (uint32_t)(RAM_FAT_MOCK_DATA_START+ADC_RAM_INFORMATION_OFFSET)
#define ADC_RAM_EXT_TRIGGER_ADDRESS             (uint32_t)(RAM_FAT_MOCK_DATA_START+ADC_RAM_EXT_TRIGGER_OFFSET)
#define ADC_RAM_MEASUREMENT_INITIAL_ADDRESS     (uint32_t)(RAM_FAT_MOCK_DATA_START+ADC_RAM_MEASUREMENT_INITIAL_OFFSET)
///////////////////////////////////////////////////

/*
    NO_TRIGGER  -   Will capture from the beggining.
    CH1_TRIGGER -   Will trigger using CH1 level (see trigger function).
    EXT_RISE_TRIGGER -   Will trigger using the external trigger input rising edge (see trigger function).
    EXT_FALL_TRIGGER -   Will trigger using the external trigger input falling edge (see trigger function).
*/
enum oscilloscope_trigger_mode{
    NO_TRIGGER = 0x0u,
    CH1_TRIGGER = 0x1u,
    EXT_RISE_TRIGGER = 0x2u,
    EXT_FALL_TRIGGER = 0x4u,
};
/*
*/
enum oscilloscope_gain_level{
    GAIN_X1 = 0x0u,
    GAIN_X3 = 0x1u,
};

///////////////////////////////////////////////////
//SETTING MODULE SCOPE VARIABLES
static enum oscilloscope_trigger_mode trigger_mode=NO_TRIGGER;
static uint32_t trigger_level_positive=0XFF<<ADC_BASE_PIN;//disabled
static uint32_t trigger_level_negative=0X00<<ADC_BASE_PIN;//disabled
static uint32_t pretrigger_size=0;
static enum gpio_irq_level ext_trigger_edge=GPIO_IRQ_EDGE_FALL;
static uint32_t samples_qty=RAM_FAT_MOCK_FILE_SIZE-ADC_RAM_MEASUREMENT_INITIAL_OFFSET;
//static uint32_t mem_end_address=RAM_FAT_MOCK_FILE_SIZE-ADC_RAM_MEASUREMENT_INITIAL_OFFSET+ADC_RAM_MEASUREMENT_INITIAL_ADDRESS;
static uint32_t adc_freq=ADC_PIO_FREQ;
static int32_t range_uv_multiplier=1000;
///////////////////////////////////////////////////
uint8_t ext_trigger_address_mark[APS6408L_BLOCK_SIZE+16] __attribute__((aligned(sizeof(uint32_t *))));//this will be saved in the last 1024 +16 bytes ram block
uint16_t ext_trigger_address_mark_index;
#define ext_trigger_address_mark_init for(uint16_t x=0;x<sizeof(ext_trigger_address_mark);x++) ext_trigger_address_mark[x]=0XFF; ext_trigger_address_mark_index=16;
///////////////////////////////////////////////////
//ADC DMA RELATED VARIABLES
volatile uint8_t adc_buffer_a[ADC_BLOCK_SIZE+16] __attribute__((aligned(sizeof(uint32_t *))));
volatile uint8_t adc_buffer_b[ADC_BLOCK_SIZE+16] __attribute__((aligned(sizeof(uint32_t *))));
volatile uint8_t *adc_ptr_a;
volatile uint8_t *adc_ptr_b;
int dma_chan_a;//channel buffer a
int dma_chan_control_a;//channel a control
int dma_chan_b;//channel buffer b
int dma_chan_control_b;//channel b control
uint dma_mem;
dma_channel_config cc;
static uint offset;
///////////////////////////////////////////////////

void machine_oscilloscope_init(void){
    //init adc read pio
    offset = pio_add_program(pio0, &pio_adc_read_program);
    pio_adc_read_program_init(pio0, ADC_SM, offset, ADC_BASE_PIN, ADC_BUS_WIDTH, ADC_CLK_PIN, ADC_PIO_FREQ);

    //init zero cross detector pin
    gpio_init(ADC_EXT_TRIGGER_PIN);
    gpio_set_dir(ADC_EXT_TRIGGER_PIN, GPIO_IN);
    gpio_init(ADC_VERTICAL_RESOLUTION_PIN);
    gpio_set_dir(ADC_VERTICAL_RESOLUTION_PIN, GPIO_IN);
        
    //Memory settings
    aps6408l_init();
    
    dma_mem = dma_claim_unused_channel(true);
    cc= dma_channel_get_default_config(dma_mem);
    channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
    channel_config_set_read_increment(&cc, true);
    channel_config_set_write_increment(&cc, false);
    channel_config_set_dreq(&cc, pio_get_dreq(pio0, APS6408L_WRITE_SM, true));
    channel_config_set_irq_quiet(&cc ,false);

    adc_buffer_a[0] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_SYNC_WRITE_BURST);
    adc_buffer_a[1] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_SYNC_WRITE_BURST);
    adc_buffer_b[0] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_SYNC_WRITE_BURST);
    adc_buffer_b[1] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_SYNC_WRITE_BURST);

    //initialize data control pointers
    adc_ptr_a = &adc_buffer_a[16];
    adc_ptr_b = &adc_buffer_b[16];
    
    // Claim 4 dma channels to dos the chain
    dma_chan_a = dma_claim_unused_channel(true);
    dma_chan_control_a = dma_claim_unused_channel(true);
    dma_chan_b = dma_claim_unused_channel(true);
    dma_chan_control_b = dma_claim_unused_channel(true);

    // Tell the chan_a and chan_b to raise IRQ line 0 when the channel finishes a block
    //dma_channel_set_irq0_enabled(dma_chan_a, true);
    //dma_channel_set_irq0_enabled(dma_chan_control_a, true);
    //dma_channel_set_irq0_enabled(dma_chan_b, true);
    //dma_channel_set_irq0_enabled(dma_chan_control_b, true);

    //uint32_t ints = save_and_disable_interrupts();
    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    //irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    //irq_set_enabled(DMA_IRQ_0, false);
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
    //  |   dma_chan_control_b
    //  |   (re-set dma_chan_b)
    //  |       |
    //  |       V
    //  |   dma_chan_b
    //  |   (fill buffer b)            
    //  |       |---->DMA_IRQ_0---->Address++ Set@bufferA
    //  |       |
    //  |   Multitrigger--->dma_memory_b
    //  |       |                    
    //  |       V
    //  |   dma_chan_control_a         
    //  |   (re-set dma_chan_a)        
    //  |-------|                      
    //      
    dma_channel_config c;

    c= dma_channel_get_default_config(dma_chan_control_a);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_irq_quiet(&c ,false);
    channel_config_set_chain_to(&c, dma_chan_a);
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
    channel_config_set_chain_to(&c, dma_chan_control_a);
    channel_config_set_high_priority( &c, true);
    dma_channel_configure(
        dma_chan_b,
        &c,
        &adc_buffer_b[16],               // Write address 
        &pio0_hw->rxf[ADC_SM],
        ADC_BLOCK_SIZE/4,           // Write the same value many times
        false                       // Don't start yet
    );

    c= dma_channel_get_default_config(dma_chan_control_b);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_irq_quiet(&c ,false);
    channel_config_set_chain_to(&c, dma_chan_b);
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
    channel_config_set_chain_to(&c, dma_chan_control_b);
    channel_config_set_high_priority( &c, true);
    dma_channel_configure(
        dma_chan_a,
        &c,
        &adc_buffer_a[16],               // Write address 
        &pio0_hw->rxf[ADC_SM],
        ADC_BLOCK_SIZE/4,           // Write the same value many times
        //false                       // trigger now
        true                       // trigger now
    );
    return;

 }

/*
    Function: range
    Set voltage range (vertical max value), will be used to cast trigger values.
    Parameter: a_obj: mVpp
    Return: mV by tar.
    Example:
        import oscilloscope
        oscilloscope.rate(3300)
        12
        
*/
STATIC mp_obj_t range(mp_obj_t a_obj) {
    // Extract the ints from the micropython input objects.
    range_uv_multiplier = (mp_obj_get_int(a_obj)*1000)/255;

    return mp_obj_new_int(range_uv_multiplier/1000);
}
// Define a Python reference to the function above.
STATIC MP_DEFINE_CONST_FUN_OBJ_1(range_obj, range);


/*
    Function: sample
    Modify adquisition rate and sample qty.
    Parameter:  a_obj: (20 - 100) @ MS/s
                b_obj: (2 - 8168) @ 1024 block
    Return: [rate value stablished in S/s,samples quantity].
    Example:
        import oscilloscope
        oscilloscope.rate(100,9000)
        [100000000,8361984]
*/
//STATIC mp_obj_t sample(mp_obj_t a_obj, mp_obj_t b_obj) {
STATIC mp_obj_t sample(size_t n_args, const mp_obj_t *args) {
    // Extract the ints from the micropython input objects.
    adc_freq = mp_obj_get_int(args[0]);    

    if(adc_freq>ADC_MAX_SAMPLE_RATE) adc_freq=ADC_MAX_SAMPLE_RATE;
    if(adc_freq<ADC_MIN_SAMPLE_RATE) adc_freq=ADC_MIN_SAMPLE_RATE;

    adc_freq*=ADC_PIO_FREQ/100;

    pio_adc_read_program_init(pio0, ADC_SM, offset, ADC_BASE_PIN, ADC_BUS_WIDTH, ADC_CLK_PIN, (float)adc_freq);

    if (n_args > 1)
    {
        uint32_t qty = mp_obj_get_int(args[1]);
        //only odd
        qty>>=1;
        qty<<=1;

        qty*=ADC_BLOCK_SIZE;

        if(qty<(2*ADC_BLOCK_SIZE)) qty=(2*ADC_BLOCK_SIZE);
        if(qty>RAM_FAT_MOCK_FILE_SIZE-ADC_RAM_MEASUREMENT_INITIAL_OFFSET) qty=RAM_FAT_MOCK_FILE_SIZE-ADC_RAM_MEASUREMENT_INITIAL_OFFSET;

        samples_qty=qty;
    }

    mp_obj_t ret_obj[2];
    ret_obj[0]=mp_obj_new_int(adc_freq/2);
    ret_obj[1]=mp_obj_new_int(samples_qty);

    return mp_obj_new_list(2,ret_obj);
}
// Define a Python reference to the function above.
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sample_obj, 1, 2, sample);
//STATIC MP_DEFINE_CONST_FUN_OBJ_2(sample_obj, sample);

/*
    Function: trigger
    Modify trigger conditions.
    Parameter: 
        a_obj:  mode (see enum oscilloscope_trigger_mode)
                modes can be combined using | operator.
        b_obj:  trigger level [(mv+),(mv-)] (see .range())
        c_obj:  pretrigger size [0-8168] @ 1024 block      
    Return: [trigger_mode, +triggerlevel tars, -trigger level tars, pretrigger_size]
    Example:
        import oscilloscope
        oscilloscope.trigger(oscilloscope.EXT_FALL_TRIGGER|oscilloscope.CH1,[40,-30],0)
        [5,168,98,0]
*/
STATIC mp_obj_t trigger(size_t n_args, const mp_obj_t *args) {
    // Extract the ints from the micropython input objects.
    trigger_mode = (enum oscilloscope_trigger_mode)mp_obj_get_int(args[0]);//trigger

    trigger_level_positive=0XFF;
    trigger_level_negative=0X00;
    if (n_args > 1)//if trigger level
    {
        size_t len;
        mp_obj_t *ptr_obj;
        mp_obj_get_array(args[1], &len, &ptr_obj);   
        if(len>0)
            trigger_level_positive=(uint32_t)((mp_obj_get_int(ptr_obj[0])*1000)/range_uv_multiplier+ADC_VIRTUAL_GROUND_LEVEL);
        if(len>1)
            trigger_level_negative=(uint32_t)((mp_obj_get_int(ptr_obj[1])*1000)/range_uv_multiplier+ADC_VIRTUAL_GROUND_LEVEL);

        if(trigger_level_positive>=0xFF||!(trigger_mode&CH1_TRIGGER)) trigger_level_positive=0XFF;//will never fire
        else if(trigger_level_positive<ADC_VIRTUAL_GROUND_LEVEL) trigger_level_positive=ADC_VIRTUAL_GROUND_LEVEL;//will always fire
        if(trigger_level_negative<=0x00||!(trigger_mode&CH1_TRIGGER)) trigger_level_negative=0X00;//will never fire
        else if(trigger_level_negative>ADC_VIRTUAL_GROUND_LEVEL) trigger_level_negative=ADC_VIRTUAL_GROUND_LEVEL;//will always fire
    }    
    trigger_level_positive<<=ADC_BASE_PIN;
    trigger_level_negative<<=ADC_BASE_PIN;
    
    ext_trigger_edge=0;
    if(trigger_mode&EXT_RISE_TRIGGER) ext_trigger_edge|=GPIO_IRQ_EDGE_RISE;
    if(trigger_mode&EXT_FALL_TRIGGER) ext_trigger_edge|=GPIO_IRQ_EDGE_FALL;
    
    pretrigger_size=0;
    if (n_args > 2)//if pretrigger
    {
        pretrigger_size=(uint32_t)mp_obj_get_int(args[2]);
        //only odd
        pretrigger_size>>=1;   
        pretrigger_size<<=1;

        pretrigger_size*=ADC_BLOCK_SIZE;

        if(pretrigger_size>RAM_FAT_MOCK_FILE_SIZE-ADC_RAM_MEASUREMENT_INITIAL_OFFSET) pretrigger_size=RAM_FAT_MOCK_FILE_SIZE-ADC_RAM_MEASUREMENT_INITIAL_OFFSET;
    }

    mp_obj_t ret_obj[4];
    ret_obj[0]=mp_obj_new_int(trigger_mode);
    ret_obj[1]=mp_obj_new_int(trigger_level_positive>>ADC_BASE_PIN);
    ret_obj[2]=mp_obj_new_int(trigger_level_negative>>ADC_BASE_PIN);
    ret_obj[3]=mp_obj_new_int(pretrigger_size);

    return mp_obj_new_list(4,ret_obj);
}
// Define a Python reference to the function above.
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(trigger_obj, 1, 3, trigger);

/*
    Function: capture
    Do an aquisition and save it in data.bin.
    Parameter: 
        a_obj:  gain (see enum oscilloscope_gain_level)
    Return: Quantity of ext_triggers that raised condition meanwhile measure.
    Example:
        import oscilloscope
        oscilloscope.capture(oscilloscope.GAIN_X1)
        0
*/
STATIC mp_obj_t capture(mp_obj_t a_obj) {
    // Extract the ints from the micropython input objects.
    ext_trigger_address_mark_init

    enum oscilloscope_gain_level gain = (enum oscilloscope_gain_level)mp_obj_get_int(a_obj);//gain

    if(gain>GAIN_X3) gain=GAIN_X3;

    //gain (vertical resolution)
    if(gain==GAIN_X3) 
    {
        gpio_set_dir(ADC_VERTICAL_RESOLUTION_PIN, GPIO_OUT);
        gpio_put(ADC_VERTICAL_RESOLUTION_PIN,0);
    }
    else gpio_set_dir(ADC_VERTICAL_RESOLUTION_PIN, GPIO_IN);

    //mp_sched_lock();
    uint32_t ints = save_and_disable_interrupts();
    aps6408l_init_mocked_file_content();
    aps6408l_burst_init();
    dma_start_channel_mask(1u << dma_chan_control_a);//trigger now
    //while(dma_channel_is_busy(dma_chan_b)==false);
    while(!(dma_hw->intr & (1u<<dma_chan_b)));//discard the first loop
    dma_hw->ints0 = 1u << dma_chan_a;
    dma_hw->ints0 = 1u << dma_chan_b;
    gpio_set_irq_enabled(ADC_EXT_TRIGGER_PIN, ext_trigger_edge, true);
    gpio_acknowledge_irq(ADC_EXT_TRIGGER_PIN, ext_trigger_edge);

    adc_buffer_a[0] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_SYNC_WRITE_BURST);
    adc_buffer_a[1] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_SYNC_WRITE_BURST);
    adc_buffer_b[0] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_SYNC_WRITE_BURST);
    adc_buffer_b[1] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_SYNC_WRITE_BURST);

    //uint32_t ini_address=ADC_RAM_MEASUREMENT_INITIAL_BLOCK;
    uint8_t triggered=false;
    uint32_t mem_address=ADC_RAM_MEASUREMENT_INITIAL_ADDRESS;
    uint32_t mem_end_address=samples_qty+ADC_RAM_MEASUREMENT_INITIAL_ADDRESS;
    uint32_t mem_address_trigger_offset=ADC_RAM_MEASUREMENT_INITIAL_ADDRESS;
    uint32_t samples_counter=0;
    uint32_t pretrigger_blocker=APS6408L_BLOCK_SIZE*8;//discard always the first 8 blocks

    if(pretrigger_size!=0) 
    {
        samples_counter+=pretrigger_size;
        pretrigger_blocker=pretrigger_size;//we will need to wait at least this samples before trigger
    }

    do
    {
        while(!(dma_hw->intr & (1u<<dma_chan_a)));
        dma_hw->ints0 = 1u << dma_chan_a;
        //dma_hw->ints0&=~(0x00000001<<dma_chan_a);
        //while(dma_channel_is_busy(dma_chan_a)==false);
        //dma_channel_wait_for_finish_blocking(dma_chan_a);
        adc_buffer_a[2] = APS6408L_DISORDER_PINS((mem_address&0xFF000000)>>24);
        adc_buffer_a[3] = APS6408L_DISORDER_PINS((mem_address&0x00FF0000)>>16);
        adc_buffer_a[4] = APS6408L_DISORDER_PINS((mem_address&0x0000FF00)>>8);
        adc_buffer_a[5] = APS6408L_DISORDER_PINS((mem_address&0x000000FF));
        gpio_set_mask(1<<APS6408L_CE_PIN);
        gpio_clr_mask(1<<APS6408L_CE_PIN);
        //gpio_put(APS6408L_CE_PIN,1);
        //gpio_put(APS6408L_CE_PIN,0);
        dma_channel_configure(
            dma_mem,
            &cc,
            &pio0_hw->txf[APS6408L_WRITE_SM],              //write
            adc_buffer_a,               //read 
            1040/4,                     // Write the same value many times
            true                       // start
        );
        while(dma_channel_is_busy(dma_mem)==true)//it will refresh the internall MP machine
        {
            uint32_t masked_val=gpio_get_all()&ADC_BUS_MASK;   
            if(masked_val>trigger_level_positive
            ||  masked_val<trigger_level_negative) triggered=true;
        }
        while(pio_sm_is_tx_fifo_empty(pio0, APS6408L_WRITE_SM)!=true);
        while(!(dma_hw->intr & (1u<<dma_chan_b)));
        
        dma_hw->ints0 = 1u << dma_chan_b;
        //dma_hw->ints0&=~(0x00000001<<dma_chan_b);
        //while(dma_channel_is_busy(dma_chan_b)==false);
        //dma_channel_wait_for_finish_blocking(dma_chan_b);
        adc_buffer_b[2] = APS6408L_DISORDER_PINS(((mem_address+APS6408L_BLOCK_SIZE)&0xFF000000)>>24);
        adc_buffer_b[3] = APS6408L_DISORDER_PINS(((mem_address+APS6408L_BLOCK_SIZE)&0x00FF0000)>>16);
        adc_buffer_b[4] = APS6408L_DISORDER_PINS(((mem_address+APS6408L_BLOCK_SIZE)&0x0000FF00)>>8);
        adc_buffer_b[5] = APS6408L_DISORDER_PINS(((mem_address+APS6408L_BLOCK_SIZE)&0x000000FF));
        gpio_set_mask(1<<APS6408L_CE_PIN);
        gpio_clr_mask(1<<APS6408L_CE_PIN);
        //gpio_put(APS6408L_CE_PIN,1);
        //gpio_put(APS6408L_CE_PIN,0);
        dma_channel_configure(
            dma_mem,
            &cc,
            &pio0_hw->txf[APS6408L_WRITE_SM],              //write
            adc_buffer_b,               //read 
            1040/4,                     // Write the same value many times
            true                       // start
        );            
        while(dma_channel_is_busy(dma_mem)==true)//it will refresh the internall MP machine
        {
            uint32_t masked_val=gpio_get_all()&ADC_BUS_MASK;   
            if(masked_val>trigger_level_positive
            ||  masked_val<trigger_level_negative) triggered=true;
        }
        while(pio_sm_is_tx_fifo_empty(pio0, APS6408L_WRITE_SM)!=true);
           
        //if external trigger   
        if(trigger_mode&(EXT_RISE_TRIGGER|EXT_FALL_TRIGGER))
        {
            if(pretrigger_blocker) gpio_acknowledge_irq(ADC_EXT_TRIGGER_PIN, ext_trigger_edge);
            else if(iobank0_hw->intr[ADC_EXT_TRIGGER_PIN / 8] & (ext_trigger_edge << (4 * (ADC_EXT_TRIGGER_PIN % 8))))
            {
                triggered=true;
                gpio_acknowledge_irq(ADC_EXT_TRIGGER_PIN, ext_trigger_edge);
                *(uint32_t*)(&ext_trigger_address_mark[ext_trigger_address_mark_index])=mem_address-ADC_RAM_MEASUREMENT_INITIAL_ADDRESS;
                if(ext_trigger_address_mark_index<sizeof(ext_trigger_address_mark)) ext_trigger_address_mark_index+=sizeof(uint32_t);//32 bits ++
            }
        }

        if(mem_address<(mem_end_address-(APS6408L_BLOCK_SIZE*2))) mem_address+=(APS6408L_BLOCK_SIZE*2);
        else mem_address=ADC_RAM_MEASUREMENT_INITIAL_ADDRESS;

        if(pretrigger_blocker) 
        {
            pretrigger_blocker-=(APS6408L_BLOCK_SIZE*2);
            triggered=false;
        }
        else if(trigger_mode==0x00) triggered=true;//fire
    
        if(triggered==true) 
        {
            samples_counter+=(APS6408L_BLOCK_SIZE*2);
        }else 
        {   
            //if no pretrigger keep always at the beggining
            if(pretrigger_size==0) mem_address=ADC_RAM_MEASUREMENT_INITIAL_ADDRESS;
            //if pretrigger saves the offset
            else mem_address_trigger_offset=mem_address;
        }
    }while(samples_counter<samples_qty); 
    dma_hw->abort = (1 << dma_chan_a) | (1 << dma_chan_control_a) | (1 << dma_chan_b) | (1 << dma_chan_control_b);
    aps6408l_burst_end();   
    
    uint8_t info_buff[1024];
    for(uint16_t x=0;x<sizeof(info_buff);x++) info_buff[x]=0;
    //save info
    //samples qty
    *(uint32_t*)(&info_buff[0])=(samples_qty);
    //rate sample
    *(uint32_t*)(&info_buff[4])=adc_freq/2;
    //positive trigger level
    *(uint32_t*)(&info_buff[8])=trigger_level_positive>>ADC_BASE_PIN;
    //positive trigger level
    *(uint32_t*)(&info_buff[12])=trigger_level_negative>>ADC_BASE_PIN;
    //trigger mode
    *(uint32_t*)(&info_buff[16])=(uint32_t)(trigger_mode);
    //pretrigger size
    *(uint32_t*)(&info_buff[20])=(uint32_t)(pretrigger_size);
    //pretrigger offset
    mem_address_trigger_offset-=ADC_RAM_MEASUREMENT_INITIAL_ADDRESS;
    if(pretrigger_size<=mem_address_trigger_offset) *(uint32_t*)(&info_buff[24])=(uint32_t)(mem_address_trigger_offset-pretrigger_size);
    else *(uint32_t*)(&info_buff[24])=(uint32_t)(samples_qty+mem_address_trigger_offset-pretrigger_size);
    //range
    *(uint32_t*)(&info_buff[28])=(uint32_t)(range_uv_multiplier);
    //gain
    *(uint32_t*)(&info_buff[32])=(uint32_t)(gain);
        
    aps6408l_write(ADC_RAM_INFORMATION_ADDRESS,info_buff);
    //save ext trigger info
    aps6408l_write(ADC_RAM_EXT_TRIGGER_ADDRESS,(uint8_t *)&ext_trigger_address_mark[16]);

    //mp_sched_unlock();            
    restore_interrupts(ints);

    return mp_obj_new_int((ext_trigger_address_mark_index-16)/sizeof(uint32_t));
}
// Define a Python reference to the function above.
STATIC MP_DEFINE_CONST_FUN_OBJ_1(capture_obj, capture);

// Define all properties of the module.
// Table entries are key/value pairs of the attribute name (a string)
// and the MicroPython object reference.
// All identifiers and strings are written as MP_QSTR_xxx and will be
// optimized to word-sized integers by the build system (interned strings).
STATIC const mp_rom_map_elem_t module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_oscilloscope) },
    { MP_ROM_QSTR(MP_QSTR_capture), MP_ROM_PTR(&capture_obj) },
    { MP_ROM_QSTR(MP_QSTR_trigger), MP_ROM_PTR(&trigger_obj) },
    { MP_ROM_QSTR(MP_QSTR_sample), MP_ROM_PTR(&sample_obj) },
    { MP_ROM_QSTR(MP_QSTR_range), MP_ROM_PTR(&range_obj) },
    { MP_ROM_QSTR(MP_QSTR_NO_TRIGGER), MP_ROM_INT(NO_TRIGGER) },
    { MP_ROM_QSTR(MP_QSTR_CH1_TRIGGER), MP_ROM_INT(CH1_TRIGGER) },
    { MP_ROM_QSTR(MP_QSTR_EXT_RISE_TRIGGER), MP_ROM_INT(EXT_RISE_TRIGGER) },
    { MP_ROM_QSTR(MP_QSTR_EXT_FALL_TRIGGER), MP_ROM_INT(EXT_FALL_TRIGGER) },
    { MP_ROM_QSTR(MP_QSTR_GAIN_X1), MP_ROM_INT(GAIN_X1) },
    { MP_ROM_QSTR(MP_QSTR_GAIN_X3), MP_ROM_INT(GAIN_X3) },
};
STATIC MP_DEFINE_CONST_DICT(module_globals, module_globals_table);

// Define module object.
const mp_obj_module_t user_oscilloscope = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&module_globals,
};

// Register the module to make it available in Python.
MP_REGISTER_MODULE(MP_QSTR_oscilloscope, user_oscilloscope);
