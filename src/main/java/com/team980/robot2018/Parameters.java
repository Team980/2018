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

    public static final double LIFT_MOTOR_MAX_UPWARD_SPEED = 0.85;
    public static final double LIFT_MOTOR_MAX_DOWNWARD_SPEED = 0.7;

    public static final double LIFT_MOTOR_MAX_MANUAL_SPEED = 0.99;

    public static final double LIFT_MOTOR_CURRENT_THRESHOLD = 20.0;

    public static final int LIFT_SYSTEM_POSITION_DEADBAND = 2000;

    // LIFT ENCODER
    public static final int LIFT_ENCODER_DIO_CHANNEL_A = 1;
    public static final int LIFT_ENCODER_DIO_CHANNEL_B = 2;
    public static final boolean INVERT_LIFT_ENCODER = false;

    public static final double LIFT_ENCODER_BOTTOM_DISTANCE = 0;
    public static final double LIFT_ENCODER_AUTO_DISTANCE = 54000;
    public static final double LIFT_ENCODER_SWITCH_DISTANCE = 58000; //Used in teleop //todo tune and increase by 1ft?
    public static final double LIFT_ENCODER_NO_SHIFT_THRESHOLD = 67000;
    public static final double LIFT_ENCODER_SCALE_DISTANCE = 182000;
    //public static final double LIFT_ENCODER_TOP_DISTANCE = 187000; //TODO soft stop?

    // CLIMBER
    public static final int CLIMB_MOTOR_CAN_ID = 0;

    // IMU
    public static final int IMU_CAN_ID = 0;

    // PNEUMATICS
    public static final int PCM_CAN_ID = 0;
    public static final int SHIFTER_SOLENOID_CHANNEL = 0;
    public static final int CLAW_SOLENOID_CHANNEL = 1;

    // AUTONOMOUS
    public static final double AUTO_MAX_SPEED = 0.5;

    public static final double AUTO_STARTING_DISTANCE = 1.0;
    public static final double AUTO_POSITIONING_DISTANCE = 5.0;

    public static final double AUTO_PAST_TARGET_DISTANCE = 2.5; //TODO tune
    public static final double AUTO_PAST_SWITCH_DISTANCE = 7.5; //TODO and re-enable

    public static final double AUTO_FAILSAFE_DISTANCE = 5.0;

    public static final double AUTO_ALLIANCE_SCALE_DISTANCE = 20; //TODO tune

    public static final double AUTO_DRIVE_FORWARD_DISTANCE = 7.5;

    public static final int AUTO_ANGULAR_SPEED_FACTOR = 45;
    public static final int AUTO_ANGULAR_DEADBAND = 5;

    public static final int AUTO_LEFT_SIDE_TURN_ANGLE = -32;
    public static final int AUTO_RIGHT_SIDE_TURN_ANGLE = 32;
    public static final int AUTO_CENTER_LEFT_TURN_ANGLE = 50; //TODO tune these
    public static final int AUTO_CENTER_RIGHT_TURN_ANGLE = -50; //TODO on practice field

}
