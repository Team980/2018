package com.team980.robot2018;

public class Parameters {

    // INPUT
    public static final int DRIVE_STICK_JS_ID = 0;
    public static final int DRIVE_WHEEL_JS_ID = 1;
    public static final int OPERATOR_CONTROLLER_JS_ID = 2;
    public static final int PRAJ_BOX_JS_ID = 3;

    // DRIVE SYSTEM
    public static final int LEFT_FRONT_DRIVE_CAN_ID = 7;
    public static final int LEFT_BACK_DRIVE_CAN_ID = 10;
    public static final int RIGHT_FRONT_DRIVE_CAN_ID = 12;
    public static final int RIGHT_BACK_DRIVE_CAN_ID = 13;

    public static final double UPPER_SHIFT_THRESHOLD = 3.4;
    public static final double LOWER_SHIFT_THRESHOLD = 2.5;

    // DRIVE ENCODERS
    public static final int LEFT_DRIVE_ENCODER_DIO_CHANNEL_A = 0;
    public static final int LEFT_DRIVE_ENCODER_DIO_CHANNEL_B = 1;
    public static final boolean INVERT_LEFT_DRIVE_ENCODER = false;

    public static final int RIGHT_DRIVE_ENCODER_DIO_CHANNEL_A = 2;
    public static final int RIGHT_DRIVE_ENCODER_DIO_CHANNEL_B = 3;
    public static final boolean INVERT_RIGHT_DRIVE_ENCODER = true;

    // LIFT SYSTEM
    public static final int LIFT_MOTOR_CAN_ID = 14;

    public static final double LIFT_MOTOR_MIN_UPWARD_SPEED = 0.3;
    public static final double LIFT_MOTOR_UPWARD_ACCELERATION = 0.01;

    public static final double LIFT_MOTOR_MAX_UPWARD_SPEED = 1.0;
    public static final double LIFT_MOTOR_MAX_DOWNWARD_SPEED = 0.5;

    public static final double LIFT_MOTOR_MAX_MANUAL_SPEED = 1.0;

    public static final double LIFT_MOTOR_CURRENT_THRESHOLD = 20.0;

    public static final double LIFT_SYSTEM_POSITION_DEADBAND = 0.01; //TODO configure

    // LIFT ENCODER
    public static final int LIFT_ENCODER_DIO_CHANNEL_A = 8;
    public static final int LIFT_ENCODER_DIO_CHANNEL_B = 9;
    public static final boolean INVERT_LIFT_ENCODER = false;

    public static final double LIFT_ENCODER_BOTTOM_DISTANCE = 0;
    public static final double LIFT_ENCODER_AUTO_DISTANCE = 1.3;
    public static final double LIFT_ENCODER_SWITCH_DISTANCE = 1.05;
    public static final double LIFT_ENCODER_NO_SHIFT_THRESHOLD = 2.28;
    public static final double LIFT_ENCODER_SCALE_DISTANCE = 2.3;
    public static final double LIFT_ENCODER_TOP_DISTANCE = 2.6; //Soft stop

    // CLIMBER
    public static final int CLIMB_MOTOR_CAN_ID = 9;

    // IMU
    public static final int IMU_CAN_ID = 6;

    // PNEUMATICS
    public static final int PCM_CAN_ID = 1;
    // Solenoid channels are hardcoded.

    // AUTONOMOUS
    public static final double AUTO_MAX_SPEED = 1.0;
    public static final double AUTO_SLOW_SPEED = 0.5;
    public static final double AUTO_SUPER_SLOW_SPEED = 0.25;

    public static final double AUTO_STARTING_DISTANCE = 1.0;
    public static final double AUTO_POSITIONING_DISTANCE = 3.0;

    public static final double AUTO_PAST_TARGET_DISTANCE = 2.5;
    public static final double AUTO_PAST_SWITCH_DISTANCE = 7.5;

    public static final double AUTO_FAILSAFE_DISTANCE = 5.0;

    public static final double AUTO_SWITCH_DISTANCE = 13.0;

    public static final double AUTO_PAST_SWITCH_SHORT_DISTANCE = 6.0;

    public static final double AUTO_NULL_ZONE_DISTANCE = 20.5;
    public static final double AUTO_APPROACH_DISTANCE = 3.5;

    public static final double AUTO_DRIVE_FORWARD_DISTANCE = 9.75;

    public static final int AUTO_ANGULAR_SPEED_FACTOR = 45;
    public static final int AUTO_ANGULAR_DEADBAND = 5;

    public static final int AUTO_TIP_CORRECTION_DELAY = 100; //in milliseconds
    public static final int AUTO_CUBE_DROP_DELAY = 400; //in milliseconds

    public static final int AUTO_CENTER_LEFT_SWITCH_TURN_ANGLE = 50;
    public static final int AUTO_CENTER_RIGHT_SWITCH_TURN_ANGLE = -50;

    public static final int AUTO_LEFT_SWITCH_TURN_ANGLE = -90;
    public static final int AUTO_RIGHT_SWITCH_TURN_ANGLE = 90;

    public static final int AUTO_LEFT_SCALE_TURN_ANGLE = -38;
    public static final int AUTO_RIGHT_SCALE_TURN_ANGLE = 38;
}
