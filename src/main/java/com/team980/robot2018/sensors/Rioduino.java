package com.team980.robot2018.sensors;

import edu.wpi.first.wpilibj.I2C;

import java.nio.ByteBuffer;

/**
 * Communicates with a Rioduino connected to the RoboRIO via MXP.
 * Also shares the received data with very easy to use getter methods.
 */
public class Rioduino {

    // LOCAL PARAMETERS
    private static final int DEVICE_ADDRESS = 4;
    private static final int BUFFER_SIZE = 20;

    private I2C arduino;

    // Schema
    private int visionTargetCoord;
    private int powerCubeWidth;
    private int powerCubeHeight;
    private int powerCubeCoord;
    private int rangedDistance;

    public Rioduino() {
        arduino = new I2C(I2C.Port.kMXP, DEVICE_ADDRESS);
        //TODO safety to check for arduino and have backup plan
    }

    public void updateData() {
        ByteBuffer buffer = ByteBuffer.allocate(BUFFER_SIZE);

        arduino.readOnly(buffer, BUFFER_SIZE);

        visionTargetCoord = buffer.getInt(0);
        powerCubeWidth = buffer.getInt(4);
        powerCubeHeight = buffer.getInt(8);
        powerCubeCoord = buffer.getInt(12);
        rangedDistance = buffer.getInt(16);
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
     * The width of the largest (i.e. closest) detected power cube
     * This should be checked to make sure it's a single cube
     * 0 if no cubes are detected
     */
    public int getPowerCubeWidth() {
        return powerCubeWidth;
    }

    /**
     * The height of the largest (i.e. closest) detected power cube
     * This should be checked to make sure it's a single cube
     * 0 if no cubes are detected
     */
    public int getPowerCubeHeight() {
        return powerCubeHeight;
    }

    /**
     * The center x-coordinate of the largest (i.e. closest) detected power cube
     * Ranges from zero to 319
     * -1 if no cubes are detected
     */
    public int getPowerCubeCoord() {
        return powerCubeCoord;
    }

    /**
     * The distance reported by the range finder
     * Zero if not set
     */
    public int getRangedDistance() {
        return rangedDistance;
    }
}
