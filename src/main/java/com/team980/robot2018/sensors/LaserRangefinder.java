/*
 *  MIT License
 *
 *  Copyright (c) 2018-2019 FRC Team 980 ThunderBots
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

package com.team980.robot2018.sensors;


import edu.wpi.first.wpilibj.I2C;

/**
 * Communicates with a laser rangefinder via the RoboRIO's I2C bus.
 * Also shares the received data with very easy to use getter methods.
 */
@Deprecated //Removed from robot
public class LaserRangefinder {

    // LOCAL PARAMETERS
    private static final int DEVICE_ADDRESS = 0x4C;
    private static final long BIT_MASK = 8188;

    private I2C sensor;

    // Schema
    private long distance;

    public LaserRangefinder() {
        sensor = new I2C(I2C.Port.kOnboard, DEVICE_ADDRESS);

        // Complex initialization sequence
        send(new byte[] {0x00, 0x04});
        send(new byte[] {0x1C, 0x65});
        send(new byte[] {0x15, 0x05});
        send(new byte[] {0x04, (byte) 0x91});
        send(new byte[] {0x15, 0x06});
        send(new byte[] {0x04, (byte) 0x92});
        send(new byte[] {0x0C, (byte) 0xE1, 0x00});
        send(new byte[] {0x0E, 0x10, (byte) 0xFF});
        send(new byte[] {0x20, 0x07, (byte) 0xD0});
        send(new byte[] {0x22, 0x50, 0x08});
        send(new byte[] {0x24, (byte) 0xA0, 0x41});
        send(new byte[] {0x26, 0x45, (byte) 0xD4});
        send(new byte[] {0x04, (byte) 0x90});
        send(new byte[] {0x15, 0x06});
        send(new byte[] {0x04, (byte) 0x92});
        send(new byte[] {0x04, (byte) 0x81});
    }

    public void updateData() {
        send(new byte[] {0x04, (byte) 0x81});
        send(new byte[] {0x00});

        byte[] isReady = receive(1);

        //Check if data is ready
        if (getBit(isReady[0], 4) == 1) {
            send(new byte[] {0x08});

            byte[] data = receive(2);
            distance = (data[1] << 8) + data[0]; //Fit the two stored bytes into the 16bit long in the correct order.

            //Check if our Data is Valid
            if (getBit(data[1], 5) == 0 && getBit(data[1], 6) == 0 && getBit(data[1], 7) == 1) {
                //Mask off and Shift off bits 13:15 and 0:1 respectively
                distance = distance & BIT_MASK;
                distance = distance >> 2;
            } else {
                distance = -1;
            }
        }
    }

    /**
     *  The distance reported by the sensor
     *  -1 if an error occurred
     */
    public long getDistance() {
        return distance;
    }

    private void send(byte[] out) {
        sensor.writeBulk(out, out.length);
    }

    private byte[] receive(int length) {
        byte[] buffer = new byte[length];
        sensor.readOnly(buffer, length);
        return buffer;
    }

    private byte getBit(byte b, int position) {
        return (byte) ((b >>> position) & 1);
    }
}
