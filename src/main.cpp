#include <Arduino.h>
#include <Wire.h>
#include <i2c_API.h>
#include <PCA9554.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <RangeSensor.h>

#include <common/mavlink.h>        // Mavlink interface
// #include <ardupilotmega/mavlink.h>        // Mavlink interface
#include <common/mavlink_msg_distance_sensor.h>

#define Serial SerialUSB
#define PCA9554B 0x20
#define DEFAULT_ADDRESS_VL53Lxx 0x29

#define VL53L1_IDENTIFICATION__MODEL_ID           0x010F


const uint16_t Scale = 10;
#define bRate 1500000

// #define TEST
// #define MAV_TEST

RangeSensor *sensors[9];
uint8_t NEW_address[9] = {0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29};
uint8_t orientation_bit[8] = {7,6,5,4,3,2,1,0};
uint8_t Address_VL53Lxx = DEFAULT_ADDRESS_VL53Lxx;

//uint8_t VL53Lxx_type = 0; //VL53L0x
uint8_t VL53Lxx_type = 1; //VL53L1x

extern void mavlink_heartbeat();
extern void init_Type(uint8_t _type, uint8_t _idx, uint8_t _i2cAddress);
extern void link_Type(uint8_t _type, uint8_t _idx, uint8_t _i2cAddress);

void setup() {
        Wire.begin();
        Serial.begin(bRate);

        Pca9554.begin();

        // Set pin mode of pins output
        Pca9554.pinMode(0, OUTPUT);
        Pca9554.pinMode(1, OUTPUT);
        Pca9554.pinMode(2, OUTPUT);
        Pca9554.pinMode(3, OUTPUT);
        Pca9554.pinMode(4, OUTPUT);
        Pca9554.pinMode(5, OUTPUT);
        Pca9554.pinMode(6, OUTPUT);
        Pca9554.pinMode(7, OUTPUT);

        // Make them low Shutdown pins of VL53L0X/VL53L1X
        Pca9554.digitalWrite(0, LOW);
        Pca9554.digitalWrite(1, LOW);
        Pca9554.digitalWrite(2, LOW);
        Pca9554.digitalWrite(3, LOW);
        Pca9554.digitalWrite(4, LOW);
        Pca9554.digitalWrite(5, LOW);
        Pca9554.digitalWrite(6, LOW);
        Pca9554.digitalWrite(7, LOW);
        delay(5000);

        // test address 0x21
        if (Address_test(NEW_address[0])) {
                Address_VL53Lxx = NEW_address[0];
                VL53Lxx_type = RangeSensor::TYP_Detect(Address_VL53Lxx);
                if (VL53Lxx_type) {
                        link_Type(VL53Lxx_type, 0, Address_VL53Lxx);
                        sensors[0]->startContinuous(50);
                }
                else{
                        init_Type(VL53Lxx_type, 0, Address_VL53Lxx);
                }

        }
        else{
                VL53Lxx_type = RangeSensor::TYP_Detect(Address_VL53Lxx);
                init_Type(VL53Lxx_type, 0, Address_VL53Lxx);
                sensors[0]->setAddress(NEW_address[0]);
                if (VL53Lxx_type) sensors[0]->startContinuous(50);
        }
#ifdef TEST
        Serial.print("\nType: ");
        Serial.print(VL53Lxx_type);
        sensors[0]->setMeasurementTimingBudget(50000);                                                         //200
        sensors[0]->startContinuous(50);
#endif
        // if (0) {
        if (Address_test(PCA9554B)) {
                Address_VL53Lxx = DEFAULT_ADDRESS_VL53Lxx;

                for (uint8_t idx = 1; idx < 9; idx++) { //9
                        Pca9554.digitalWrite(orientation_bit[idx-1], HIGH);
                        delay(100);

                        if (Address_test(Address_VL53Lxx)) {

                                init_Type(VL53Lxx_type, idx, Address_VL53Lxx);
                                sensors[idx]->setAddress(NEW_address[idx]);
                                if (VL53Lxx_type) sensors[idx]->startContinuous(50);
#ifdef TEST
                                sensors[idx]->setMeasurementTimingBudget(50000); //20000
                                sensors[idx]->startContinuous(50);
#endif
                        }
                }
        }
}

void loop() {

        mavlink_heartbeat();
        // delay(2000);
        float SensorSmooth;
        for (uint8_t idx = 0; idx < 9; idx++) {
                if(sensors[idx]) {
#ifdef TEST
                        delay(1000);
                        Serial.print("\nsysid ");
                        Serial.print(sensors[idx]->mavlink_d.id);
                        // Serial.print(" orientation ");
                        // Serial.print(sensors[idx]->mavlink_d.orientation);
                        Serial.print(" Address_VL53Lxx ");
                        Serial.print(sensors[idx]->address,HEX);
                        Serial.print(" Range: ");
                        Serial.print(sensors[idx]->readRangeContinuousMillimeters());
                        if (sensors[idx]->timeoutOccurred()) {
                                Serial.print("\n TIMEOUT");
                        }
#endif
                        // SensorSmooth = sensors[idx]->readRangeContinuousMillimeters();
                        // sensors[idx]->timeoutOccurred();
                        SensorSmooth = sensors[idx]->readRangeSingleMillimeters();
                        // Serial.print(SensorSmooth);
                        SensorSmooth = constrain(SensorSmooth, (sensors[idx]->mavlink_d.min_distance*Scale), (sensors[idx]->mavlink_d.max_distance*Scale));
                        sensors[idx]->mavlink_d.current_distance = SensorSmooth / Scale;
#ifdef MAV_TEST
                        Serial.print(" Range: ");
                        Serial.print(sensors[idx]->readRangeSingleMillimeters());
                        Serial.print(" orientation ");
                        Serial.print(sensors[idx]->mavlink_d.orientation);
                        Serial.print(" distance: ");
                        Serial.print(sensors[idx]->mavlink_d.current_distance);
                        Serial.print("\n");
#endif
                        // Initialize the required buffers
                        mavlink_message_t msg;
                        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                        // Pack the message
                        mavlink_msg_distance_sensor_pack(sensors[idx]->mavlink_d.sysid, \
                                                         sensors[idx]->mavlink_d.compid, \
                                                         &msg, \
                                                         sensors[idx]->mavlink_d.time_boot_ms, \
                                                         sensors[idx]->mavlink_d.min_distance, \
                                                         sensors[idx]->mavlink_d.max_distance, \
                                                         sensors[idx]->mavlink_d.current_distance, \
                                                         sensors[idx]->mavlink_d.type, \
                                                         sensors[idx]->mavlink_d.id, \
                                                         sensors[idx]->mavlink_d.orientation, \
                                                         sensors[idx]->mavlink_d.covariance);
                        // Copy the message to the send buffer
                        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
                        // Send the message (.write sends as bytes)
                        Serial.write(buf, len);
                }
        }
}

void mavlink_heartbeat() {

        //< ID 1 for this system
        uint8_t system_id = 100;
        //< The component sending the message.
        uint8_t component_id = MAV_COMP_ID_PATHPLANNER;
        // Initialize the required buffers
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        // Define the system type, in this case ground control station
        uint8_t type = MAV_TYPE_GCS;
        uint8_t autopilot = MAV_AUTOPILOT_INVALID;
        uint8_t base_mode = 0;
        uint32_t custom_mode = 0;
        uint8_t system_status = 0;
        // Pack the message
        mavlink_msg_heartbeat_pack(system_id, \
                                   component_id, \
                                   &msg, \
                                   type, \
                                   autopilot, \
                                   base_mode, \
                                   custom_mode, \
                                   system_status);
        // Copy the message to the send buffer
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        // Send the message
        Serial.write(buf, len);
}

void init_Type(uint8_t _type, uint8_t _idx, uint8_t _i2cAddress) {

        if (_type == 0) {
                sensors[_idx] = new VL53L0X(_i2cAddress);
                sensors[_idx]->init_MAVLINK(_type,_idx);
                sensors[_idx]->init(true);
                sensors[_idx]->setTimeout(sensors[_idx]->mavlink_d.Timeout);
#ifndef TEST
                VL53L0X *aas;
                aas = (VL53L0X *)sensors[_idx];
                aas->setSignalRateLimit(0.25);
                aas->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, sensors[_idx]->mavlink_d.PreRng);
                aas->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, sensors[_idx]->mavlink_d.PostRng);
#endif
        }

        if (_type == 1) {
                sensors[_idx] = new VL53L1X(_i2cAddress);
                sensors[_idx]->init_MAVLINK(_type,_idx);
                sensors[_idx]->init(true);
                sensors[_idx]->setTimeout(sensors[_idx]->mavlink_d.Timeout);

                VL53L1X *aas;
                aas = (VL53L1X *)sensors[_idx];
                aas->setDistanceMode(VL53L1X::Long);
        }
}

void link_Type(uint8_t _type, uint8_t _idx, uint8_t _i2cAddress) {

        if (_type == 0) {
                sensors[_idx] = new VL53L0X(_i2cAddress);
        }
        if (_type == 1) {
                sensors[_idx] = new VL53L1X(_i2cAddress);
        }
        sensors[_idx]->init_MAVLINK(_type,_idx);
        sensors[_idx]->setTimeout(sensors[_idx]->mavlink_d.Timeout);
}
