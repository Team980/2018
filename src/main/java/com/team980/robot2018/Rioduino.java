package com.team980.robot2018;

import edu.wpi.first.wpilibj.I2C;

import java.nio.ByteBuffer;

/**
 * Communicates with a Rioduino connected to the RoboRIO via MXP.
 * Also shares the received data with very easy to use getter methods.
 */
public class Rioduino {

    // LOCAL PARAMETERS
    private static final int DEVICE_ADDRESS = 4;
    private static final int BUFFER_SIZE = 8;

    private I2C arduino;

    // Schema
    private int visionTargetCoord;
    private int rangedDistance;

    public Rioduino() {
        arduino = new I2C(I2C.Port.kMXP, DEVICE_ADDRESS);
        //TODO safety to check for arduino and have backup plan
    }

    public void updateData() {
        ByteBuffer buffer = ByteBuffer.allocate(BUFFER_SIZE);

        arduino.readOnly(buffer, BUFFER_SIZE);

        visionTargetCoord = buffer.getInt(0);
        rangedDistance = buffer.getInt(4);
    }

    /**
     * The center x-coordinate of the detected vision target
     * Ranges from zero to 319
     * -1 if no target is detected
     */
    public int getVisionTargetCoord() {
        return visionTargetCoord;
    }

    /**
     * The distance reported by the range finder
     * Zero if not set
     */
    public int getRangedDistance() {
        return rangedDistance;
    }
}
