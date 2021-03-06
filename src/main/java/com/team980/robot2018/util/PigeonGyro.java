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
