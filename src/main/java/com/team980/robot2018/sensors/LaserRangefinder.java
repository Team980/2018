package com.team980.robot2018.sensors;


import edu.wpi.first.wpilibj.I2C;

/**
 * Communicates with a laser rangefinder via the RoboRIO's I2C bus.
 * Also shares the received data with very easy to use getter methods.
 */
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
            distance = (data[0] << 8) + data[1]; //Fit the two stored bytes into the 16bit int in the correct order.

            System.out.println("A: " + data[1]);
            System.out.println("B: " + data[2]);
            System.out.println("C: " + distance);

            //Check if our Data is Valid
            if (getBit(data[0], 5) == 0 && getBit(data[0], 6) == 0 && getBit(data[0], 7) == 1) {
                //Mask off and Shift off bits 13:15 and 0:1 respectively
                distance = distance & BIT_MASK;
                System.out.println("D: " + distance);
                distance = distance >> 2;
            } else {
                distance = -1;
            }

            System.out.println("E: " + distance);
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
        sensor.transaction(out, out.length, new byte[] {}, 0);
    }

    private byte[] receive(int length) {
        byte[] in = new byte[length];
        sensor.transaction(new byte[] {}, 0, in, in.length);
        return in;
    }

    private byte getBit(byte b, int position) {
        return (byte) ((b >>> position) & 1);
    }
}
