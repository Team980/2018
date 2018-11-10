/*
 *  MIT License
 *
 *  Copyright (c) 2018 FRC Team 980 ThunderBots
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

package com.team980.robot2018;

public class Parameters {

    // INPUT
    public static final int DRIVE_STICK_JS_ID = 0;
    public static final int DRIVE_WHEEL_JS_ID = 1;
    public static final int OPERATOR_CONTROLLER_JS_ID = 2;
    public static final int PRAJ_BOX_JS_ID = 3;

    // DRIVE SYSTEM
    public static final int LEFT_FRONT_DRIVE_CAN_ID = 1;
    public static final int LEFT_BACK_DRIVE_CAN_ID = 4;
    public static final int LEFT_TOP_DRIVE_CAN_ID = 3;

    public static final int RIGHT_FRONT_DRIVE_CAN_ID = 0;
    public static final int RIGHT_BACK_DRIVE_CAN_ID = 5;
    public static final int RIGHT_TOP_DRIVE_CAN_ID = 2;

    public static final double UPPER_SHIFT_THRESHOLD = 4;
    public static final double LOWER_SHIFT_THRESHOLD = 2.5;

    // DRIVE ENCODERS
    public static final int LEFT_DRIVE_ENCODER_DIO_CHANNEL_A = 7;
    public static final int LEFT_DRIVE_ENCODER_DIO_CHANNEL_B = 8;
    public static final boolean INVERT_LEFT_DRIVE_ENCODER = false;

    public static final int RIGHT_DRIVE_ENCODER_DIO_CHANNEL_A = 4;
    public static final int RIGHT_DRIVE_ENCODER_DIO_CHANNEL_B = 5;
    public static final boolean INVERT_RIGHT_DRIVE_ENCODER = true;

    // DRIVE PID
    public static final boolean DRIVE_PID_ENABLED = false;
    public static final double PID_MAX_SPEED_LOW_GEAR = 5.5;
    public static final double PID_MAX_SPEED_HIGH_GEAR = 18;

    public static final double LEFT_DRIVE_P = 0.0;
    public static final double LEFT_DRIVE_I = 0.0;
    public static final double LEFT_DRIVE_D = 0.0;

    public static final double RIGHT_DRIVE_P = 0.0;
    public static final double RIGHT_DRIVE_I = 0.0;
    public static final double RIGHT_DRIVE_D = 0.0;

    // LIFT SYSTEM
    public static final int LIFT_MOTOR_CAN_ID = 1;

    public static final double LIFT_MOTOR_MIN_UPWARD_SPEED = 0.1;
    public static final double LIFT_MOTOR_UPWARD_ACCELERATION = 0.04;

    public static final double LIFT_MOTOR_MAX_UPWARD_SPEED = 1.0;
    public static final double LIFT_MOTOR_MAX_DOWNWARD_SPEED = 0.7;

    public static final double LIFT_MOTOR_MAX_MANUAL_SPEED = 0.99;

    public static final double LIFT_MOTOR_CURRENT_THRESHOLD = 20.0;

    public static final int LIFT_SYSTEM_POSITION_DEADBAND = 6000;

    // LIFT ENCODER
    public static final int LIFT_ENCODER_DIO_CHANNEL_A = 1;
    public static final int LIFT_ENCODER_DIO_CHANNEL_B = 2;
    public static final boolean INVERT_LIFT_ENCODER = false;

    public static final double LIFT_ENCODER_BOTTOM_DISTANCE = 0;
    public static final double LIFT_ENCODER_AUTO_DISTANCE = 61000;
    public static final double LIFT_ENCODER_SWITCH_DISTANCE = 58000; //Used in teleop //todo tune and increase by 1ft?
    public static final double LIFT_ENCODER_NO_SHIFT_THRESHOLD = 67000;
    public static final double LIFT_ENCODER_SCALE_DISTANCE = 188000;
    public static final double LIFT_ENCODER_TOP_DISTANCE = 190000; //soft stop

    // CLIMBER
    public static final int CLIMB_MOTOR_CAN_ID = 0;

    // IMU
    public static final int IMU_CAN_ID = 0;

    // PNEUMATICS
    public static final int PCM_CAN_ID = 0;
    public static final int SHIFTER_SOLENOID_CHANNEL = 0;
    public static final int CLAW_SOLENOID_CHANNEL = 1;

    // AUTONOMOUS
    public static final double AUTO_MAX_SPEED = 1.0;

    public static final double AUTO_SWITCH_SPEED = 0.75;
    public static final double AUTO_SWITCH_MIN_SPEED = 0.30;

    public static final double AUTO_SCALE_SPEED = 1.00;
    public static final double AUTO_SCALE_MIN_SPEED = 0.75;

    public static final double AUTO_SCALE_BACKUP_SPEED = 0.60;

    public static final double AUTO_HIGH_GEAR_SPEED = 0.90; //0.60;
    public static final double AUTO_HIGH_GEAR_MIN_SPEED = 0.20; //0.30;

    public static final double AUTO_STARTING_DISTANCE = 1.0;
    public static final double AUTO_POSITIONING_DISTANCE = 6.5;
    public static final double AUTO_SWITCH_APPROACH_DISTANCE = 7.0;

    public static final double AUTO_PAST_TARGET_DISTANCE = 5; //TODO used to be 2.5
    public static final double AUTO_PAST_SWITCH_DISTANCE = 9.5; //TODO used to be 7.5

    public static final double AUTO_FAILSAFE_DISTANCE = 5.0;

    public static final double AUTO_SWITCH_DISTANCE = 13.0;
    public static final double AUTO_SWITCH_SIDE_APPROACH_DISTANCE = 3.0;

    public static final double AUTO_PAST_SWITCH_SHORT_DISTANCE = 6.0;

    public static final double AUTO_NULL_ZONE_DISTANCE = 18.50;
    public static final double AUTO_SCALE_APPROACH_DISTANCE = 4.25;
    public static final double AUTO_SCALE_REVERSE_DISTANCE = 2.0;

    public static final double AUTO_MOVE_TO_CUBE_DISTANCE = 1.0;

    public static final double AUTO_FIELD_RUN_DISTANCE = 18.2;
    public static final double AUTO_CROSS_FIELD_DISTANCE = 5.0; //TODO 18; The cable protector kills our encoders
    public static final double AUTO_CROSS_FIELD_STOP_THRESHOLD = 82.0; // - so let's use our rangefinder for a dirty hack
    public static final double AUTO_DIRECT_APPROACH_DISTANCE = 5.5;

    public static final double AUTO_DRIVE_FORWARD_DISTANCE = 9.75;

    public static final int AUTO_HIGH_GEAR_ANGULAR_SPEED_FACTOR = 30;

    public static final double AUTO_TURN_SPEED = 0.40;
    public static final double AUTO_TURN_MIN_SPEED = 0.20;

    public static final int AUTO_ANGULAR_SPEED_FACTOR = 45;
    public static final int AUTO_ANGULAR_DEADBAND = 5;

    public static final int AUTO_POST_DROP_DELAY = 400; //in milliseconds
    public static final int AUTO_TIP_CORRECTION_DELAY = 100; //in milliseconds

    public static final int AUTO_CENTER_LEFT_SWITCH_TURN_ANGLE = 50;
    public static final int AUTO_CENTER_RIGHT_SWITCH_TURN_ANGLE = -50;

    public static final int AUTO_LEFT_SWITCH_TURN_ANGLE = -90;
    public static final int AUTO_RIGHT_SWITCH_TURN_ANGLE = 90;

    public static final int AUTO_LEFT_SCALE_TURN_ANGLE = -38;
    public static final int AUTO_RIGHT_SCALE_TURN_ANGLE = 38;

}
