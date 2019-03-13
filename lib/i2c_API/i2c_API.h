#ifndef __I2C_API_CLASS_H
#define __I2C_API_CLASS_H

#include <Arduino.h>
#include <Wire.h>

uint16_t readRegister(uint8_t i2cAddress, uint8_t reg);
void writeRegister(uint8_t i2cAddress, uint8_t reg, uint8_t value);
void writeReg16_8(uint8_t i2cAddress, uint16_t reg, uint8_t value);
uint16_t readReg16Bit(uint8_t i2cAddress, uint16_t reg);
uint8_t Address_test(uint8_t i2cAddress);

#endif /* __RANGE_SENSOR_CLASS_H */
