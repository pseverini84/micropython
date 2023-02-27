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

#ifndef _APS6408L_H
#define _APS6408L_H

#include <stdio.h>
void aps6408l_init(void);
void aps6408l_burst_init(void);
void aps6408l_burst_end(void);
void aps6408l_write(uint32_t address, uint8_t *buffer);
void aps6408l_read(uint32_t address, uint8_t *buffer);
void aps6408l_init_mocked_file_content(void);

#define APS6408L_WRITE_SM 1
#define APS6408L_BLOCK_SIZE (uint32_t)1024//1Kbyte
#define APS6408L_BYTE_SIZE (uint32_t)(8*1024*APS6408L_BLOCK_SIZE)//8Mbytes
#define APS6408L_PIO_FREQ 250000000
#define APS6408L_BASE_PIN 18
#define APS6408L_CLK_PIN 26
#define APS6408L_DQS_PIN 27
#define APS6408L_CE_PIN 28
#define APS6408L_RST_PIN 29
#define APS6408L_BUS_WIDTH 8
#define APS6408L_BUS_MASK (uint32_t)0X000000FF<<APS6408L_BASE_PIN

extern const uint8_t aps6408l_lut_disorder[];
extern const uint8_t aps6408l_lut_order[];
//#define APS6408L_DISORDER_PINS(ordered) aps6408l_lut_disorder[ordered]//(uint32_t)(((ordered&0x01)<<4)|((ordered&0x02)<<2)|((ordered&0x04)<<4)|((ordered&0x08)<<2)|((ordered&0x10)<<3)|((ordered&0xE0)>>5))
//#define APS6408L_ORDER_PINS(disordered) aps6408l_lut_order[disordered]//(uint32_t)(((disordered&0x07)<<5)|((disordered&0x08)>>2)|((disordered&0x10)>>4)|((disordered&0x20)>>2)|((disordered&0x40)>>4)|((disordered&0x80)>>3))
#define APS6408L_DISORDER_PINS(ordered) ordered
#define APS6408L_ORDER_PINS(disordered) disordered

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

#endif
