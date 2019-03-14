This code for Arduino allows the use of the [9xVL53LXX](https://github.com/ipmgroup/RangeFinder_9xVL53LXX_hardware) sensor array via MAVLink.

# Usage
The following instructions are for testing with [ArduPilot](https://github.com/ArduPilot/ardupilot)
1. Connect the sensor array via I2C to SparkFun SAMD21 Mini Breakout running this code (any other Arduino board should work).
2. Connect the SparkFun SAMD21 Mini Breakout via USB to the board running ArduPilot.
3. Set the parameters in ArduPilot as follows:
   - `SERIAL2_BAUD 1500`
   - `SERIAL2_PROTOCOL 1`
   - `PRX_TYPE 2`
   - `RNGFND1_TYPE 10`
   - `RNGFND1_ORIENT 25`
4. Restart ArduPilot with the option `-D` (if you configured `SERIAL2` in step 2, otherwise adjust according to table below) and point it to the serial port that your MAVLink board is connected to. For example: `sudo ./arducopter -A udp:192.168.0.100:14550 -D /dev/ttyACM0`

Select options and serial ports in steps 2 and 3 according to this table:

Serial | Option
-------|-------
SERIAL0|-A
SERIAL1|-C
SERIAL2|-D
SERIAL3|-B
SERIAL4|-E
SERIAL5|-F
SERIAL6|-G
