package com.team980.robot2018;

public class Parameters {

    // INPUT
    public static final int DRIVE_STICK_JS_ID = 0;
    public static final int DRIVE_WHEEL_JS_ID = 1;
    public static final int OPERATOR_CONTROLLER_JS_ID = 2;

    // DRIVE SYSTEM
    public static final int LEFT_FRONT_DRIVE_CAN_ID = 1;
    public static final int LEFT_BACK_DRIVE_CAN_ID = 4;
    public static final int LEFT_TOP_DRIVE_CAN_ID = 3;

    public static final int RIGHT_FRONT_DRIVE_CAN_ID = 0;
    public static final int RIGHT_BACK_DRIVE_CAN_ID = 5;
    public static final int RIGHT_TOP_DRIVE_CAN_ID = 2;

    public static final double UPPER_SHIFT_THRESHOLD = 3.25;
    public static final double LOWER_SHIFT_THRESHOLD = 2.5;

    // DRIVE ENCODERS
    public static final int LEFT_DRIVE_ENCODER_DIO_CHANNEL_A = 7;
    public static final int LEFT_DRIVE_ENCODER_DIO_CHANNEL_B = 8;
    public static final boolean INVERT_LEFT_DRIVE_ENCODER = false;

    public static final int RIGHT_DRIVE_ENCODER_DIO_CHANNEL_A = 4;
    public static final int RIGHT_DRIVE_ENCODER_DIO_CHANNEL_B = 5;
    public static final boolean INVERT_RIGHT_DRIVE_ENCODER = true;

    // LIFT SYSTEM
    public static final int LIFT_MOTOR_CAN_ID = 0;

    public static final double LIFT_MOTOR_MIN_UPWARD_SPEED = 0.1; //0.3; TODO tune
    public static final double LIFT_MOTOR_UPWARD_ACCELERATION = 0.01;

    public static final double LIFT_MOTOR_MAX_UPWARD_SPEED = 0.6; //TODO tune these
    public static final double LIFT_MOTOR_MAX_DOWNWARD_SPEED = 0.5;

    public static final double LIFT_MOTOR_MAX_MANUAL_SPEED = 0.6;

    public static final int LIFT_MOTOR_PDP_CHANNEL = 15;
    public static final double LIFT_MOTOR_CURRENT_THRESHOLD = 20.0;

    // LIFT ENCODER
    public static final int LIFT_ENCODER_DIO_CHANNEL_A = 1;
    public static final int LIFT_ENCODER_DIO_CHANNEL_B = 2;
    public static final boolean INVERT_LIFT_ENCODER = false;

    public static final double LIFT_ENCODER_BOTTOM_DISTANCE = 0;
    public static final double LIFT_ENCODER_SWITCH_DISTANCE = 58000;
    public static final double LIFT_ENCODER_SCALE_DISTANCE = 182000;
    public static final double LIFT_ENCODER_TOP_DISTANCE = 187000; //TODO soft stop

    // IMU
    public static final int IMU_CAN_ID = -1; //TODO

    // PNEUMATICS
    public static final int PCM_CAN_ID = 0;
    public static final int SHIFTER_SOLENOID_CHANNEL = 0;
    public static final int CLAW_SOLENOID_CHANNEL = 1;

    // AUTONOMOUS
    public static final double AUTO_MAX_SPEED = 0.4; //TODO increase

    public static final double AUTO_STARTING_DISTANCE = 1.0;
    public static final double AUTO_POSITIONING_DISTANCE = 3.0;
    public static final double AUTO_BACKUP_DISTANCE = 5.0;

    public static final double AUTO_PAST_TARGET_DISTANCE = 2.5;
    public static final double AUTO_PAST_SWITCH_DISTANCE = 7.5;

    public static final double AUTO_ALLIANCE_SCALE_DISTANCE = 22; //TODO tune
    public static final double AUTO_OPPONENT_SCALE_DISTANCE = 18; //TODO on trash panda

    public static final int AUTO_ANGULAR_SPEED_FACTOR = 45;
    public static final int AUTO_ANGULAR_DEADBAND = 5;

    public static final int AUTO_LEFT_SIDE_TURN_ANGLE = -32; //TODO tune these at competition
    public static final int AUTO_RIGHT_SIDE_TURN_ANGLE = 32;
    public static final int AUTO_CENTER_LEFT_TURN_ANGLE = 42;
    public static final int AUTO_CENTER_RIGHT_TURN_ANGLE = -42;

}
