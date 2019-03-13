#ifndef __RANGE_SENSOR_CLASS_H
#define __RANGE_SENSOR_CLASS_H

#define VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N   0xBF
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID    0xC2
#define SOFT_RESET_VL53L1X 0x0000

class RangeSensor
{
public:

struct MAVLINK_data {
        //MAVLINK DISTANCE MESSAGE
        uint8_t sysid;
        //< The component sending the message.
        uint8_t compid;

        uint32_t time_boot_ms; /*< Time since system boot*/
        uint16_t min_distance; /*< Minimum distance the sensor can measure in centimeters*/
        uint16_t max_distance; /*< Maximum distance the sensor can measure in centimeters*/
        uint16_t current_distance; /*< Current distance reading*/
        uint8_t type; /*< Type from MAV_DISTANCE_SENSOR enum.*/
        uint8_t id; /*< Onboard ID of the sensor*/
        uint8_t orientation; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
        // Consumed within ArduPilot by the proximity class

        uint8_t covariance; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/
        uint16_t Timeout;
        uint16_t PreRng;
        uint16_t PostRng;
};

struct MAVLINK_data mavlink_d;

void init_MAVLINK(int typ, int id);

const MAVLINK_data MAVLINK_VL53L0X_init[9] = {
        //sysid,compid,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance,Timeout,PreRng,PostRng
        {     1,   158,           0,           1,         170,               0,   0, 9,         25,         0,   200,     18,     14}, // 25
        {     1,   158,           0,           1,         170,               0,   0, 1,          0,         0,   200,     18,     14},
        {     1,   158,           0,           1,         170,               0,   0, 2,          1,         0,   200,     18,     14},
        {     1,   158,           0,           1,         170,               0,   0, 3,          2,         0,   200,     18,     14},
        {     1,   158,           0,           1,         170,               0,   0, 4,          3,         0,   200,     18,     14},
        {     1,   158,           0,           1,         170,               0,   0, 5,          4,         0,   200,     18,     14},
        {     1,   158,           0,           1,         170,               0,   0, 6,          5,         0,   200,     18,     14},
        {     1,   158,           0,           1,         170,               0,   0, 7,          6,         0,   200,     18,     14},
        {     1,   158,           0,           1,         170,               0,   0, 8,          7,         0,   200,     18,     14}
};

//static const
const MAVLINK_data MAVLINK_VL53L1X_init[9] = {
        //sysid,compid,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance,Timeout,PreRng,PostRng
        {     1,   158,           0,           1,         400,               0,   0, 9,         25,         0,   200,     18,     14}, //25
        {     1,   158,           0,           1,         400,               0,   0, 1,          0,         0,   200,     18,     14},
        {     1,   158,           0,           1,         400,               0,   0, 2,          1,         0,   200,     18,     14},
        {     1,   158,           0,           1,         400,               0,   0, 3,          2,         0,   200,     18,     14},
        {     1,   158,           0,           1,         400,               0,   0, 4,          3,         0,   200,     18,     14},
        {     1,   158,           0,           1,         400,               0,   0, 5,          4,         0,   200,     18,     14},
        {     1,   158,           0,           1,         400,               0,   0, 6,          5,         0,   200,     18,     14},
        {     1,   158,           0,           1,         400,               0,   0, 7,          6,         0,   200,     18,     14},
        {     1,   158,           0,           1,         400,               0,   0, 8,          7,         0,   200,     18,     14}
};

RangeSensor(uint8_t);

enum TYP_DetectType { VL53L0, VL53L1};
static uint8_t TYP_Detect(uint8_t i2cAddress);
static void Reset_VL53L0(uint8_t i2cAddress);
static void Reset_VL53L1(uint8_t i2cAddress);


virtual bool init(bool io_2v8 = true) = 0;
virtual void setTimeout(uint16_t timeout) = 0;
virtual void startContinuous(uint32_t period_ms) = 0;
virtual uint16_t readRangeContinuousMillimeters(void) = 0;
virtual uint16_t readRangeSingleMillimeters(void) = 0;
virtual bool timeoutOccurred(void) = 0;
virtual bool setMeasurementTimingBudget(uint32_t budget_us) = 0;
virtual void setAddress(uint8_t new_addr) = 0;

uint8_t address;

};

#endif /* __RANGE_SENSOR_CLASS_H */
