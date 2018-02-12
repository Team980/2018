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
    public static final int LEFT_DRIVE_ENCODER_DIO_CHANNEL_A = 0;
    public static final int LEFT_DRIVE_ENCODER_DIO_CHANNEL_B = 1;
    public static final boolean INVERT_LEFT_DRIVE_ENCODER = false;

    public static final int RIGHT_DRIVE_ENCODER_DIO_CHANNEL_A = 2;
    public static final int RIGHT_DRIVE_ENCODER_DIO_CHANNEL_B = 3;
    public static final boolean INVERT_RIGHT_DRIVE_ENCODER = true;
}
