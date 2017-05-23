#ifndef TPS22994_H
#define TPS22994_H

#include <string.h>
#include <nrf_drv_twi.h>

// Address is actually 0xE0 but only the 7 LSB counts. (bangs head into desk)
//E0 E2 E4 E6 E8 EA EC
#define TPS22994_I2C_ADDRESS        (0xE0>>1)
#define TPS22994_REG_CONTROL        0x05

void tps22994_init(uint8_t address, nrf_drv_twi_t* twidriver);
void tps22994_channel_off(uint8_t address, uint8_t channel);
void tps22994_channel_on(uint8_t address, uint8_t channel);
void tps22994_channel_toggle(uint8_t address, uint8_t channel);
uint8_t tps22994_read_ctrl_reg(uint8_t address);
uint8_t tps22994_read_reg(uint8_t address, uint8_t reg);
void tps22994_write_ctrl_reg(uint8_t address, uint8_t value);
void tps22994_write_reg(uint8_t address, uint8_t reg, uint8_t value);

#endif
