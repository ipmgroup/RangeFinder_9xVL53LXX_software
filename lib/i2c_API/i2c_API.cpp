// #include <Arduino.h>
// #include <Wire.h>
#include <i2c_API.h>

/***************************************************************************
 *
 *  Writes 8-bits to the specified destination register
 *
 **************************************************************************/
void writeRegister(uint8_t i2cAddress, uint8_t reg, uint8_t value)
{
        Wire.beginTransmission(i2cAddress);
        Wire.write((uint8_t)reg);
        Wire.write((uint8_t)value);
        Wire.endTransmission();
}

// Write an 8-bit register
void writeReg16_8(uint8_t i2cAddress, uint16_t reg, uint8_t value)
{
        Wire.beginTransmission(i2cAddress);
        Wire.write((reg >> 8) & 0xFF); // reg high byte
        Wire.write( reg       & 0xFF);// reg low byte
        Wire.write(value);
        Wire.endTransmission();
}

/***************************************************************************
 *
 * Reads 8-bits from the specified source register
 *
 **************************************************************************/
uint16_t readRegister(uint8_t i2cAddress, uint8_t reg)
{
        Wire.beginTransmission(i2cAddress);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom(i2cAddress, (uint8_t)1);
        return Wire.read();
}

/***************************************************************************
 *
 * Reads 16-bits from the specified source register
 *
 **************************************************************************/
uint16_t readReg16Bit(uint8_t i2cAddress, uint16_t reg) {
        uint16_t value;

        Wire.beginTransmission(i2cAddress);
        Wire.write((reg >> 8) & 0xFF); // reg high byte
        Wire.write(reg & 0xFF);  // reg low byte
        Wire.endTransmission();

        Wire.requestFrom(i2cAddress, (uint8_t)2);
        value = (uint16_t)Wire.read() << 8; // value high byte
        value |= Wire.read();         // value low byte

        return value;
}

uint8_t Address_test(uint8_t i2cAddress) {
        uint8_t error;
        Wire.beginTransmission(i2cAddress);
        error = Wire.endTransmission();

        if (error == 0)
        {
                return 1;
        }
        else if (error==4)
        {
                return -1;
        }
        return 0;
}
