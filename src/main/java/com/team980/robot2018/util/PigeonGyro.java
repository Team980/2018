package com.team980.robot2018.util;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.GyroBase;

/**
 * Hack wrapper to interface with the Pigeon IMU as a Gyro
 * Required for Shuffleboard to represent the yaw as a gryo widget
 */
public class PigeonGyro extends GyroBase {

    private PigeonIMU backingImu;

    public PigeonGyro(PigeonIMU imu) {
        backingImu = imu;
    }

    public PigeonGyro(int canId) {
        this(new PigeonIMU(canId));
    }

    @Override
    public void calibrate() {
        backingImu.setYaw(0, 0);
    }

    @Override
    public void reset() {
        backingImu.setYaw(0, 0);
    }

    @Override
    public double getAngle() {
        double[] ypr = new double[3];
        backingImu.getYawPitchRoll(ypr);
        return ypr[0];
    }

    @Override
    public double getRate() { //N/A
        return -1;
    }
}
