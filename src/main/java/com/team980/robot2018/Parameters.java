package com.team980.robot2018;

public class Parameters {

    // INPUT
    public static final ControlMode CONTROL_MODE = ControlMode.COMPETITION_DRIVER_STATION;

    public static final int DRIVE_STICK_JS_ID = 0;
    public static final int DRIVE_WHEEL_JS_ID = 1;
    public static final int OPERATOR_CONTROLLER_JS_ID = 2;

    // DRIVE SYSTEM
    public static final int LEFT_FRONT_DRIVE_CAN_ID = 7;
    public static final int LEFT_BACK_DRIVE_CAN_ID = 10;
    public static final int RIGHT_FRONT_DRIVE_CAN_ID = 12;
    public static final int RIGHT_BACK_DRIVE_CAN_ID = 13;

    public static final double UPPER_SHIFT_THRESHOLD = 3.25;
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

    public static final int LIFT_MOTOR_PDP_CHANNEL = 11;
    public static final double LIFT_MOTOR_CURRENT_THRESHOLD = 20.0;

    // LIFT ENCODER
    public static final int LIFT_ENCODER_DIO_CHANNEL_A = 8;
    public static final int LIFT_ENCODER_DIO_CHANNEL_B = 9;
    public static final boolean INVERT_LIFT_ENCODER = false;

    // PROXIMITY SENSORS
    public static final int LOWER_PROXIMITY_SENSOR_ANALOG_CHANNEL = 1;
    public static final int UPPER_PROXIMITY_SENSOR_ANALOG_CHANNEL = 0;

    public static final double PROXIMITY_SENSOR_THRESHOLD = 2.5;

    // PHOTO SWITCH
    public static final int PHOTO_SWITCH_ANALOG_CHANNEL = 2;

    public static final double PHOTO_SWITCH_THRESHOLD = 2.5;

    // IMU
    public static final int IMU_CAN_ID = 6;

    // PNEUMATICS
    public static final int PCM_CAN_ID = 1;
    public static final int SHIFTER_SOLENOID_CHANNEL = 4;
    public static final int CLAW_SOLENOID_CHANNEL = 3;

    // RELAYS
    public static final int DALEK_EYE_RELAY_CHANNEL = 0;

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
