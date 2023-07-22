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

#include "py/runtime.h"
#include "extmod/vfs.h"
#include "modrp2.h"
#include "pico/binary_info.h"

#include <stdio.h>
#include <string.h>
#include "hardware/gpio.h"
#include "pico/time.h"
#include "aps6408l.h"
#include "ram_fat_mock.h"
#include "pio_aps6408l_write.pio.h"
#include "hardware/pio.h"
#include "pico/sem.h"

semaphore_t aps6408l_semaphore;

const uint8_t aps6408l_lut_disorder[]={
0, 16, 8, 24, 64, 80, 72, 88, 32, 48, 40, 56, 96, 112, 104, 120, 128, 144, 136, 152, 192, 208, 200, 216, 160, 176, 168, 184, 224, 240, 232, 248, 1, 17, 9, 25, 65, 81, 73, 89, 33, 49, 41, 57, 97, 113, 105, 121, 129, 145, 137, 153, 193, 209, 201, 217, 161, 177, 169, 185, 225, 241, 233, 249, 2, 18, 10, 26, 66, 82, 74, 90, 34, 50, 42, 58, 98, 114, 106, 122, 130, 146, 138, 154, 194, 210, 202, 218, 162, 178, 170, 186, 226, 242, 234, 250, 3, 19, 11, 27, 67, 83, 75, 91, 35, 51, 43, 59, 99, 115, 107, 123, 131, 147, 139, 155, 195, 211, 203, 219, 163, 179, 171, 187, 227, 243, 235, 251, 4, 20, 12, 28, 68, 84, 76, 92, 36, 52, 44, 60, 100, 116, 108, 124, 132, 148, 140, 156, 196, 212, 204, 220, 164, 180, 172, 188, 228, 244, 236, 252, 5, 21, 13, 29, 69, 85, 77, 93, 37, 53, 45, 61, 101, 117, 109, 125, 133, 149, 141, 157, 197, 213, 205, 221, 165, 181, 173, 189, 229, 245, 237, 253, 6, 22, 14, 30, 70, 86, 78, 94, 38, 54, 46, 62, 102, 118, 110, 126, 134, 150, 142, 158, 198, 214, 206, 222, 166, 182, 174, 190, 230, 246, 238, 254, 7, 23, 15, 31, 71, 87, 79, 95, 39, 55, 47, 63, 103, 119, 111, 127, 135, 151, 143, 159, 199, 215, 207, 223, 167, 183, 175, 191, 231, 247, 239, 255
};
const uint8_t aps6408l_lut_order[]={
0 ,32 ,64 ,96 ,128 ,160 ,192 ,224 ,2 ,34 ,66 ,98 ,130 ,162 ,194 ,226 ,1 ,33 ,65 ,97 ,129 ,161 ,193 ,225 ,3 ,35 ,67 ,99 ,131 ,163 ,195 ,227 ,8 ,40 ,72 ,104 ,136 ,168 ,200 ,232 ,10 ,42 ,74 ,106 ,138 ,170 ,202 ,234 ,9 ,41 ,73 ,105 ,137 ,169 ,201 ,233 ,11 ,43 ,75 ,107 ,139 ,171 ,203 ,235 ,4 ,36 ,68 ,100 ,132 ,164 ,196 ,228 ,6 ,38 ,70 ,102 ,134 ,166 ,198 ,230 ,5 ,37 ,69 ,101 ,133 ,165 ,197 ,229 ,7 ,39 ,71 ,103 ,135 ,167 ,199 ,231 ,12 ,44 ,76 ,108 ,140 ,172 ,204 ,236 ,14 ,46 ,78 ,110 ,142 ,174 ,206 ,238 ,13 ,45 ,77 ,109 ,141 ,173 ,205 ,237 ,15 ,47 ,79 ,111 ,143 ,175 ,207 ,239 ,16 ,48 ,80 ,112 ,144 ,176 ,208 ,240 ,18 ,50 ,82 ,114 ,146 ,178 ,210 ,242 ,17 ,49 ,81 ,113 ,145 ,177 ,209 ,241 ,19 ,51 ,83 ,115 ,147 ,179 ,211 ,243 ,24 ,56 ,88 ,120 ,152 ,184 ,216 ,248 ,26 ,58 ,90 ,122 ,154 ,186 ,218 ,250 ,25 ,57 ,89 ,121 ,153 ,185 ,217 ,249 ,27 ,59 ,91 ,123 ,155 ,187 ,219 ,251 ,20 ,52 ,84 ,116 ,148 ,180 ,212 ,244 ,22 ,54 ,86 ,118 ,150 ,182 ,214 ,246 ,21 ,53 ,85 ,117 ,149 ,181 ,213 ,245 ,23 ,55 ,87 ,119 ,151 ,183 ,215 ,247 ,28 ,60 ,92 ,124 ,156 ,188 ,220 ,252 ,30 ,62 ,94 ,126 ,158 ,190 ,222 ,254 ,29 ,61 ,93 ,125 ,157 ,189 ,221 ,253 ,31 ,63 ,95 ,127 ,159 ,191 ,223 ,255
};

static uint offset;

void aps6408l_init(void)
{
    sem_init (&aps6408l_semaphore, 1, 1);

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
    gpio_set_slew_rate(APS6408L_CE_PIN,GPIO_SLEW_RATE_FAST);
    gpio_set_dir(APS6408L_CE_PIN, GPIO_OUT);
    gpio_init(APS6408L_RST_PIN);
    gpio_set_dir(APS6408L_RST_PIN, GPIO_OUT);

    gpio_put(APS6408L_CLK_PIN,0);
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

    //////////////////////////////
    //load the mock FAT in order to show data.bin (8168 block size = 8364032 bytes)
    for(uint32_t x=0;x<RAM_FAT_MOCK_LEN;x+=APS6408L_BLOCK_SIZE)
        aps6408l_write(x,(uint8_t *)&ram_fat_mock[x]);
    //fill the rest of the memory with 0s
    for(uint32_t x=RAM_FAT_MOCK_LEN;x<APS6408L_BYTE_SIZE/*RAM_FAT_MOCK_DATA_START*/;x+=APS6408L_BLOCK_SIZE)
        aps6408l_write(x,(uint8_t *)&ram_fat_mock_zero[0]);
    //////////////////////////////

    offset=pio_add_program(pio0, &pio_aps6408l_write_program);
}

void aps6408l_init_mocked_file_content(void)
{
    for(uint32_t x=RAM_FAT_MOCK_LEN;x<APS6408L_BYTE_SIZE;x+=APS6408L_BLOCK_SIZE)
        aps6408l_write(x,(uint8_t *)&ram_fat_mock_zero[0]);
}

void aps6408l_burst_init(void)
{
    sem_acquire_blocking (&aps6408l_semaphore);
    pio_aps6408l_write_program_init(pio0, APS6408L_WRITE_SM, offset, APS6408L_BASE_PIN, APS6408L_BUS_WIDTH, APS6408L_CLK_PIN, APS6408L_PIO_FREQ);

    gpio_set_slew_rate(APS6408L_CE_PIN,GPIO_SLEW_RATE_FAST);
    gpio_set_dir(APS6408L_DQS_PIN, GPIO_OUT);
    gpio_put(APS6408L_DQS_PIN,0);
    gpio_put(APS6408L_CE_PIN,1);
}

void aps6408l_burst_end(void)
{
    sem_release (&aps6408l_semaphore);
    gpio_set_dir(APS6408L_DQS_PIN, GPIO_IN);
    gpio_set_dir(APS6408L_CE_PIN, GPIO_OUT);
    gpio_put(APS6408L_CE_PIN,1);
}

void aps6408l_write(uint32_t address, uint8_t *buffer)
{
    sem_acquire_blocking (&aps6408l_semaphore);
    //Memory setting
    for (int gpio = APS6408L_BASE_PIN; gpio < APS6408L_BASE_PIN + APS6408L_BUS_WIDTH; gpio++) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_OUT);
    }
    gpio_init(APS6408L_CLK_PIN);
    gpio_set_dir(APS6408L_CLK_PIN, GPIO_OUT);

    gpio_put(APS6408L_CLK_PIN,0);
    
    uint8_t command[6];
    command[0] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_SYNC_WRITE_BURST);
    command[1] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_SYNC_WRITE_BURST);
    command[2] = APS6408L_DISORDER_PINS((address&0xFF000000)>>24);
    command[3] = APS6408L_DISORDER_PINS((address&0x00FF0000)>>16);
    command[4] = APS6408L_DISORDER_PINS((address&0x0000FF00)>>8);
    command[5] = APS6408L_DISORDER_PINS((address&0x000000FF));

    gpio_set_dir(APS6408L_DQS_PIN, GPIO_IN);
    gpio_put(APS6408L_CE_PIN,0);
    for(uint8_t x=0;x<6;x+=2){
        gpio_put_masked(APS6408L_BUS_MASK,(uint32_t)command[x]<<APS6408L_BASE_PIN);
        gpio_put(APS6408L_CLK_PIN,1);
        gpio_put_masked(APS6408L_BUS_MASK,(uint32_t)command[x+1]<<APS6408L_BASE_PIN);
        gpio_put(APS6408L_CLK_PIN,0);
    }

    for(uint8_t x=0;x<5;x++){
        gpio_put(APS6408L_CLK_PIN,1);
        gpio_put(APS6408L_CLK_PIN,0);
    }


    gpio_put(APS6408L_CE_PIN,0);
    for(uint16_t x=0;x<APS6408L_BLOCK_SIZE/2;x++){
        gpio_put_masked(APS6408L_BUS_MASK,(uint32_t)*buffer++<<APS6408L_BASE_PIN);
        gpio_put(APS6408L_CLK_PIN,1);
        gpio_put_masked(APS6408L_BUS_MASK,(uint32_t)*buffer++<<APS6408L_BASE_PIN);
        gpio_put(APS6408L_CLK_PIN,0);
    }

    gpio_set_dir(APS6408L_DQS_PIN, GPIO_IN);
    gpio_put(APS6408L_CE_PIN,1);
    sem_release (&aps6408l_semaphore);
}

void aps6408l_read(uint32_t address, uint8_t *buffer)
{
    sem_acquire_blocking (&aps6408l_semaphore);
    //Memory setting
    for (int gpio = APS6408L_BASE_PIN; gpio < APS6408L_BASE_PIN + APS6408L_BUS_WIDTH; gpio++) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_OUT);
    }
    gpio_init(APS6408L_CLK_PIN);
    gpio_set_dir(APS6408L_CLK_PIN, GPIO_OUT);

    gpio_put(APS6408L_CLK_PIN,0);
    
    uint8_t command[6];
    command[0] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_SYNC_READ_BURST);
    command[1] = APS6408L_DISORDER_PINS(APS6408L_COMMAND_SYNC_READ_BURST);
    command[2] = APS6408L_DISORDER_PINS((address&0xFF000000)>>24);
    command[3] = APS6408L_DISORDER_PINS((address&0x00FF0000)>>16);
    command[4] = APS6408L_DISORDER_PINS((address&0x0000FF00)>>8);
    command[5] = APS6408L_DISORDER_PINS((address&0x000000FF));

    gpio_set_dir(APS6408L_DQS_PIN, GPIO_IN);
    gpio_put(APS6408L_CE_PIN,0);
    for(uint8_t x=0;x<6;x+=2){
        gpio_put_masked(APS6408L_BUS_MASK,(uint32_t)command[x]<<APS6408L_BASE_PIN);
        gpio_put(APS6408L_CLK_PIN,1);
        gpio_put_masked(APS6408L_BUS_MASK,(uint32_t)command[x+1]<<APS6408L_BASE_PIN);
        gpio_put(APS6408L_CLK_PIN,0);
    }

    for (int gpio = APS6408L_BASE_PIN; gpio < APS6408L_BASE_PIN + APS6408L_BUS_WIDTH; gpio++) {
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_IN);
    }

    gpio_set_dir(APS6408L_DQS_PIN, GPIO_OUT);
    gpio_put(APS6408L_DQS_PIN,0);

    for(uint8_t x=0;x<5;x++){
        gpio_put(APS6408L_CLK_PIN,1);
        gpio_put(APS6408L_CLK_PIN,0);
    }

    for(uint16_t x=0;x<APS6408L_BLOCK_SIZE/2;x++){
        gpio_put(APS6408L_CLK_PIN,1);
        for(volatile uint8_t y=0;y<2;y++);
        //sleep_us(1);
        *buffer++=(gpio_get_all()&APS6408L_BUS_MASK)>>APS6408L_BASE_PIN;
        gpio_put(APS6408L_CLK_PIN,0);
        for(volatile uint8_t y=0;y<2;y++);
        //sleep_us(1);
        *buffer++=(gpio_get_all()&APS6408L_BUS_MASK)>>APS6408L_BASE_PIN;
    }

    gpio_set_dir(APS6408L_DQS_PIN, GPIO_IN);
    gpio_put(APS6408L_CE_PIN,1);
    sem_release (&aps6408l_semaphore);
}


typedef struct _aps6408l_obj_t {
    mp_obj_base_t base;
    uint32_t aps6408l_base;
    uint32_t aps6408l_size;
} aps6408l_obj_t;

const mp_obj_type_t aps6408l_type;

STATIC aps6408l_obj_t aps6408l_obj = {
    .base = { &aps6408l_type },
    .aps6408l_base = 0,
    .aps6408l_size = APS6408L_BYTE_SIZE,
};

STATIC mp_obj_t aps6408l_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    // Check args.
    mp_arg_check_num(n_args, n_kw, 0, 0, false);

    // Return singleton object.
    return MP_OBJ_FROM_PTR(&aps6408l_obj);
}

STATIC mp_obj_t aps6408l_readblocks(size_t n_args, const mp_obj_t *args) {
    //aps6408l_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint32_t offset = mp_obj_get_int(args[1]) * APS6408L_BLOCK_SIZE;
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[2], &bufinfo, MP_BUFFER_WRITE);
    if (n_args == 4) {
        offset += mp_obj_get_int(args[3]);
    }
    aps6408l_read(offset,bufinfo.buf);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(aps6408l_readblocks_obj, 3, 4, aps6408l_readblocks);

STATIC mp_obj_t aps6408l_writeblocks(size_t n_args, const mp_obj_t *args) {
    //aps6408l_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint32_t offset = mp_obj_get_int(args[1]) * APS6408L_BLOCK_SIZE;
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[2], &bufinfo, MP_BUFFER_READ);
    aps6408l_write(offset, bufinfo.buf);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(aps6408l_writeblocks_obj, 3, 4, aps6408l_writeblocks);

STATIC mp_obj_t aps6408l_ioctl(mp_obj_t self_in, mp_obj_t cmd_in, mp_obj_t arg_in) {
    mp_int_t cmd = mp_obj_get_int(cmd_in);
    switch (cmd) {
        case MP_BLOCKDEV_IOCTL_INIT:
            return MP_OBJ_NEW_SMALL_INT(0);
        case MP_BLOCKDEV_IOCTL_DEINIT:
            return MP_OBJ_NEW_SMALL_INT(0);
        case MP_BLOCKDEV_IOCTL_SYNC:
            return MP_OBJ_NEW_SMALL_INT(0);
        case MP_BLOCKDEV_IOCTL_BLOCK_COUNT:
            return MP_OBJ_NEW_SMALL_INT(APS6408L_BYTE_SIZE / APS6408L_BLOCK_SIZE);
        case MP_BLOCKDEV_IOCTL_BLOCK_SIZE:
            return MP_OBJ_NEW_SMALL_INT(APS6408L_BLOCK_SIZE);
        case MP_BLOCKDEV_IOCTL_BLOCK_ERASE: {
            return MP_OBJ_NEW_SMALL_INT(0);
        }
        default:
            return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(aps6408l_ioctl_obj, aps6408l_ioctl);

STATIC const mp_rom_map_elem_t aps6408l_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_readblocks), MP_ROM_PTR(&aps6408l_readblocks_obj) },
    { MP_ROM_QSTR(MP_QSTR_writeblocks), MP_ROM_PTR(&aps6408l_writeblocks_obj) },
    { MP_ROM_QSTR(MP_QSTR_ioctl), MP_ROM_PTR(&aps6408l_ioctl_obj) },
};
STATIC MP_DEFINE_CONST_DICT(aps6408l_locals_dict, aps6408l_locals_dict_table);

// This defines the type(Timer) object.
MP_DEFINE_CONST_OBJ_TYPE(
    aps6408l_type,
    MP_QSTR_ram,
    MP_TYPE_FLAG_NONE,
    make_new, aps6408l_make_new,
    locals_dict, &aps6408l_locals_dict
    );

/*
const mp_obj_type_t aps6408l_type = {
    { &mp_type_type },
    .name = MP_QSTR_aps6408l,
    .make_new = aps6408l_make_new,
    .locals_dict = (mp_obj_dict_t *)&aps6408l_locals_dict,
};
*/
// Define all attributes of the module.
// Table entries are key/value pairs of the attribute name (a string)
// and the MicroPython object reference.
// All identifiers and strings are written as MP_QSTR_xxx and will be
// optimized to word-sized integers by the build system (interned strings).
STATIC const mp_rom_map_elem_t aps6408l_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR__name__),   MP_ROM_QSTR(MP_QSTR_aps6408l) },
    { MP_ROM_QSTR(MP_QSTR_ram),    MP_ROM_PTR(&aps6408l_type) },
};
STATIC MP_DEFINE_CONST_DICT(aps6408l_module_global, aps6408l_globals_table);

// Define module object.
const mp_obj_module_t user_aps6408l = {
    .base = { &mp_type_module/*aps6408l_type*/ },
    .globals = (mp_obj_dict_t *)&aps6408l_module_global,
};

// Register the module to make it available in Python.
MP_REGISTER_MODULE(MP_QSTR_aps6408l, user_aps6408l);