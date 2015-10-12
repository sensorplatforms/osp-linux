
/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_I2C_API_H
#define MBED_I2C_API_H

#include "device.h"
#include "PinNames.h"

#if DEVICE_I2C
#define RX_LENGTH   (16)

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    I2C_LPC_INVALID = -1,
    I2C_LPC_0 = 0,
    I2C_LPC_1 = 1,
    I2C_LPC_2 = 2,
    I2C_LPC_MAX
}i2c_num;

typedef struct
{
    i2c_num i2c;
    void    *px_master_handle;
    void    *px_slave_handle;
    uint32_t  ui_slave_address;
    uint8_t   uc_slave_index;
#ifdef I2CM_IRQ_BASED
    uint32_t     ui_irq_priority;
#endif /* I2CM_IRQ_BASED */
    void   *tx_buff;
  
    uint8_t rxBuff[RX_LENGTH];
    uint8_t rxCount;        /* Bytes so far received  */
    uint8_t rxLength;       /* Expected Rx buffer length */

    /*
         * i2c_operation indicates the I2C action to be performed as requested by host
         * 0x1 --> receive data
         * 0x2 --> tramsit data
         */
    uint8_t   i2c_operation;

    /* I2C transfer structure; directly manipulated by ROM driver */
    ROM_I2CS_XFER_T   pXfer; //TODO: Remove dependency on ROM struct
} i2c_t;

enum {
  I2C_ERROR_NO_SLAVE = -1,
  I2C_ERROR_BUS_BUSY = -2
};

void i2c_init(i2c_t *obj, PinName sda, PinName scl);
void i2c_frequency    (i2c_t *obj, int hz);
int  i2c_start (i2c_t *obj);
int  i2c_stop(i2c_t *obj);
int  i2c_read(i2c_t *obj, int address, char *data, int length, int stop);
int  i2c_write(i2c_t *obj, int address, const char *data, int length, int stop);
void i2c_reset(i2c_t *obj);
int  i2c_byte_read(i2c_t *obj, int last);
int  i2c_byte_write(i2c_t *obj, int data);

#if DEVICE_I2CSLAVE
void i2c_slave_mode   (i2c_t *obj, int enable_slave);
int  i2c_slave_receive(i2c_t *obj); // Wait for next I2C event and find out what is going on
int  i2c_slave_read   (i2c_t *obj, char *data, int length);
int  i2c_slave_write  (i2c_t *obj, const char *data, int length);
int  i2c_slave_byte_read(i2c_t *obj, int last);
int  i2c_slave_byte_write(i2c_t *obj, int data);
void i2c_slave_address(i2c_t *obj, int idx, uint32_t address, uint32_t mask);
#endif
void i2c_wait_completion( void );
#ifdef __cplusplus
}
#endif

#endif

#endif

