package com.team980.robot2018;

public class Parameters {

    // INPUT
    public static final ControlMode CONTROL_MODE = ControlMode.SINGLE_JOYSTICK;

    public static final int DRIVE_STICK_JS_ID = 0;
    public static final int DRIVE_WHEEL_JS_ID = 1;
    public static final int OPERATOR_BOX_JS_ID = 2;
    public static final int GAME_CONTROLLER_JS_ID = 3;

    // DRIVE SYSTEM
    public static final int LEFT_DRIVE_PWM_CHANNEL = 0;
    public static final int RIGHT_DRIVE_PWM_CHANNEL = 1;

    public static final double UPPER_SHIFT_THRESHOLD = 3.25;
    public static final double LOWER_SHIFT_THRESHOLD = 2.5;

    // ENCODERS
    public static final int LEFT_ENCODER_DIO_CHANNEL_A = 1;
    public static final int LEFT_ENCODER_DIO_CHANNEL_B = 2;
    public static final boolean INVERT_LEFT_ENCODER = false;

    public static final int RIGHT_ENCODER_DIO_CHANNEL_A = 4;
    public static final int RIGHT_ENCODER_DIO_CHANNEL_B = 5;
    public static final boolean INVERT_RIGHT_ENCODER = true;

    // IMU
    public static final int IMU_CAN_ID = 6;

    // PNEUMATICS
    public static final int PCM_CAN_ID = 1;
    public static final int SHIFTER_SOLENOID_CHANNEL = 4;

    // AUTONOMOUS
    public static final double AUTO_MAX_SPEED = 0.5;

    public static final double AUTO_STARTING_DISTANCE = 1.0;
    public static final double AUTO_POSITIONING_DISTANCE = 5.0;
    public static final double AUTO_BACKUP_DISTANCE = 10.0;

    public static final int AUTO_ANGULAR_SPEED_FACTOR = 45;
    public static final int AUTO_ANGULAR_DEADBAND = 5;

    public static final int AUTO_LEFT_SIDE_TURN_ANGLE = -32;
    public static final int AUTO_RIGHT_SIDE_TURN_ANGLE = 32;
    public static final int AUTO_CENTER_LEFT_TURN_ANGLE = 42;
    public static final int AUTO_CENTER_RIGHT_TURN_ANGLE = -42;

    public enum ControlMode {
        COMPETITION_DRIVER_STATION,
        SINGLE_JOYSTICK,
        GAME_CONTROLLER
    }
}
