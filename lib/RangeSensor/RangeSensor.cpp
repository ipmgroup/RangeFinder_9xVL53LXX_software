//#include <Wire.h>
#include <i2c_API.h>
#include <RangeSensor.h>

#define VL53L1_IDENTIFICATION__MODEL_ID           0x010F
#define ADDRESS_DEFAULT                           0b0101001
#define VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N   0xBF
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID       0xC0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID    0xC2

RangeSensor::RangeSensor(uint8_t _address = ADDRESS_DEFAULT)
        : address(_address)
        // , io_timeout(0) // no timeout
        // , did_timeout(false)
{
}

void RangeSensor::init_MAVLINK(int typ, int id){
        if (!typ) mavlink_d = MAVLINK_VL53L0X_init[id];
        if (typ) mavlink_d = MAVLINK_VL53L1X_init[id];
};

uint8_t RangeSensor::TYP_Detect(uint8_t i2cAddress){

        uint16_t f1;
        f1 = readReg16Bit(i2cAddress, VL53L1_IDENTIFICATION__MODEL_ID);
        Serial.print("\nMODEL_ID=");
        Serial.print(f1);
        if (f1 == 0xEACC) {
                // Serial.print("\nDetected VL53L1X");
                return 1;
        }

        uint8_t v1, v2;
        v1 = readRegister(i2cAddress, 0xC0);
        v2 = readRegister(i2cAddress, 0xC1);
        if (v1 == 0xEE || v2 == 0xAA) {
                // Serial.print("\nDetected VL53L0X");
                return 0;
        }

        return -1;
}

//reset VL53L0X
void RangeSensor::Reset_VL53L0(uint8_t i2cAddress){
        // Serial.print("\nDetected reset VL53L0");
        writeRegister(i2cAddress, VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x00); // reset device
        // while (readRegister(VL53Lxx, VL53L0X_REG_IDENTIFICATION_REVISION_ID) != 0x00);
        delay(100);
        writeRegister(i2cAddress, VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x01); // reset device
        //while (readRegister(VL53Lxx, VL53L0X_REG_IDENTIFICATION_REVISION_ID) != 0x00);
        delay(100);
};
//reset VL53L1X
void RangeSensor::Reset_VL53L1(uint8_t i2cAddress){
        // Serial.print("\nDetected reset VL53L1");
        writeReg16_8(i2cAddress, SOFT_RESET_VL53L1X, 0x00);
        delay(100);
        writeReg16_8(i2cAddress, SOFT_RESET_VL53L1X, 0x01);
        delay(100);
};
