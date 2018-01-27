package com.team980.robot2018.sensors;


/**
 * Communicates with a laser rangefinder via the RoboRIO's I2C bus.
 * Also shares the received data with very easy to use getter methods.
 */
public class LaserRangefinder {/* TODO make this work. It doesn't. AT ALL. NOT EVEN IN THEORY.

    // LOCAL PARAMETERS
    private static final int DEVICE_ADDRESS = 0x4C;

    private I2C sensor;

    byte dataReady = 0;

    byte distance1 = 0;
    byte distance2 = 0;
    unsigned
    int distance = 0;
    unsigned
    int mask = 8188;

    public LaserRangefinder() {
        sensor = new I2C(I2C.Port.kOnboard, DEVICE_ADDRESS);

        //INT_PAD High
        Wire.beginTransmission(DEVICE_ADDRESS); //i2c address of the device
        Wire.write(0x00); //point it to the correct register
        Wire.write(0x04); //write!
        Wire.endTransmission(); //end the message

        //byte[] b = {0x00, 0x04};
        //sensor.transaction(b, b.length, null, 0);

        //I2C Config
        Wire.beginTransmission(DEVICE_ADDRESS);
        Wire.write(0x1C);
        Wire.write(0x65);
        Wire.endTransmission();

        //Initialize Off
        Wire.beginTransmission(DEVICE_ADDRESS);
        Wire.write(0x15);
        Wire.write(0x05);
        Wire.endTransmission();

        //MCPU Off
        Wire.beginTransmission(DEVICE_ADDRESS);
        Wire.write(0x04);
        Wire.write(0x91);
        Wire.endTransmission();

        //Initialize On
        Wire.beginTransmission(DEVICE_ADDRESS);
        Wire.write(0x15);
        Wire.write(0x06);
        Wire.endTransmission();

        //MCPU On
        Wire.beginTransmission(DEVICE_ADDRESS);
        Wire.write(0x04);
        Wire.write(0x92);
        Wire.endTransmission();

        //Tof Configuration
        byte w1[] = {0xE1, 0x00};
        Wire.beginTransmission(DEVICE_ADDRESS);
        Wire.write(0x0C);
        Wire.write(w1, 2);
        Wire.endTransmission();

        byte w2[] = {0x10, 0xFF};
        Wire.beginTransmission(DEVICE_ADDRESS);
        Wire.write(0x0E);
        Wire.write(w2, 2);
        Wire.endTransmission();

        byte w3[] = {0x07, 0xD0};
        Wire.beginTransmission(DEVICE_ADDRESS);
        Wire.write(0x20);
        Wire.write(w3, 2);
        Wire.endTransmission();

        byte w4[] = {0x50, 0x08};
        Wire.beginTransmission(DEVICE_ADDRESS);
        Wire.write(0x22);
        Wire.write(w4, 2);
        Wire.endTransmission();

        byte w5[] = {0xA0, 0x41};
        Wire.beginTransmission(DEVICE_ADDRESS);
        Wire.write(0x24);
        Wire.write(w5, 2);
        Wire.endTransmission();

        byte w6[] = {0x45, 0xD4};
        Wire.beginTransmission(DEVICE_ADDRESS);
        Wire.write(0x26);
        Wire.write(w6, 2);
        Wire.endTransmission();

        //Standby State
        Wire.beginTransmission(address);
        Wire.write(0x04);
        Wire.write(0x90);
        Wire.endTransmission();

        //Set Initialization. It's laser time
        Wire.beginTransmission(DEVICE_ADDRESS); //i2c address of the device
        Wire.write(0x15); //point it to the correct register
        Wire.write(0x06); //write!

        //The module powers up in Standby Mode. Turn it On!
        Wire.beginTransmission(DEVICE_ADDRESS);
        Wire.write(0x04);
        Wire.write(0x92);
        Wire.endTransmission();

        //Single Meaure Command (For good measure)
        Wire.beginTransmission(DEVICE_ADDRESS);
        Wire.write(0x04);
        Wire.write(0x81);
        Wire.endTransmission();

    }


    public void updateData() {
        //Single Measure Command
        Wire.beginTransmission(address);
        Wire.write(0x04);
        Wire.write(0x81);
        Wire.endTransmission();

        //Point at Status register
        Wire.beginTransmission(address);
        Wire.write(0x00);
        Wire.endTransmission();

        // Receive data from the Status Register
        Wire.requestFrom(address,byte(1));
        dataReady = Wire.read();

        //Check if data is ready
        if (bitRead(dataReady, 4) == 1) {
            //Point at Result Register
            Wire.beginTransmission(address);
            Wire.write(0x08);
            Wire.endTransmission();

            //We only need bytes 2:12 of the 16 bits stored in this register
            Wire.requestFrom(address, byte(2));  // Receive 2 bytes of data from the Result Register and store in buffer
            distance2 = Wire.read();   //Take first byte (8 bits) in buffer and store it in byte distance2 (this will go at the back of out 16bit int)
            distance1 |= Wire.read();  //Read the remaining byte and put it byte distance1 (This will go at the front)
            distance = (distance1 << 8) + distance2; //Fit the two stored bytes into the 16bit int in the correct order.

            //Check if our Data is Valid
            if (bitRead(distance, 13) == 0 && bitRead(distance, 14) == 0 && bitRead(distance, 15) == 1) {
                //Mask off and Shift off bits 13:15 and 0:1 respectively
                distance = distance & mask;
                distance = distance >> 2;
            } else {
                Serial.println("Distance data error");
            }
        }

        //Reset all variables. This appears to be an important step
        distance = 0;
        distance2 = 0;
        distance1 = 0;
    }

    public int getDistance() {
        return distance;
    }*/
}
