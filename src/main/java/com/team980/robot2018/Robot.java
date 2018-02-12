package com.team980.robot2018;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends IterativeRobot { //TODO test TimedRobot - exact 20ms vs about 20ms

    private Joystick driveStick;
    private Joystick driveWheel;
    private Joystick operatorController;

    private DifferentialDrive robotDrive;

    //private Encoder leftDriveEncoder;
    //private Encoder rightDriveEncoder;

    private PowerDistributionPanel pdp;

    @Override
    public void robotInit() {
        driveStick = new Joystick(Parameters.DRIVE_STICK_JS_ID);
        driveWheel = new Joystick(Parameters.DRIVE_WHEEL_JS_ID);
        operatorController = new Joystick(Parameters.OPERATOR_CONTROLLER_JS_ID);

        WPI_VictorSPX leftTopMotor = new WPI_VictorSPX(Parameters.LEFT_TOP_DRIVE_CAN_ID);
        leftTopMotor.setInverted(true);
        SpeedControllerGroup leftDrive = new SpeedControllerGroup(new WPI_VictorSPX(Parameters.LEFT_FRONT_DRIVE_CAN_ID),
                new WPI_VictorSPX(Parameters.LEFT_BACK_DRIVE_CAN_ID),
                leftTopMotor);

        WPI_VictorSPX rightTopMotor = new WPI_VictorSPX(Parameters.RIGHT_TOP_DRIVE_CAN_ID);
        rightTopMotor.setInverted(true);
        SpeedControllerGroup rightDrive = new SpeedControllerGroup(new WPI_VictorSPX(Parameters.RIGHT_FRONT_DRIVE_CAN_ID),
                new WPI_VictorSPX(Parameters.RIGHT_BACK_DRIVE_CAN_ID),
                rightTopMotor);

        robotDrive = new DifferentialDrive(leftDrive, rightDrive);
        robotDrive.setName("Robot Drive");

        /*leftDriveEncoder = new Encoder(Parameters.LEFT_DRIVE_ENCODER_DIO_CHANNEL_A, Parameters.LEFT_DRIVE_ENCODER_DIO_CHANNEL_B, Parameters.INVERT_LEFT_DRIVE_ENCODER, CounterBase.EncodingType.k4X);
        leftDriveEncoder.setDistancePerPulse((2 * (Constants.PI) * (Constants.DRIVE_WHEEL_RADIUS / 12)) / (Constants.DRIVE_ENCODER_PULSES_PER_REVOLUTION));
        leftDriveEncoder.setName("Drive Encoders", "Left");

        rightDriveEncoder = new Encoder(Parameters.RIGHT_DRIVE_ENCODER_DIO_CHANNEL_A, Parameters.RIGHT_DRIVE_ENCODER_DIO_CHANNEL_B, Parameters.INVERT_RIGHT_DRIVE_ENCODER, CounterBase.EncodingType.k4X);
        rightDriveEncoder.setDistancePerPulse((2 * (Constants.PI) * (Constants.DRIVE_WHEEL_RADIUS / 12)) / (Constants.DRIVE_ENCODER_PULSES_PER_REVOLUTION));
        rightDriveEncoder.setName("Drive Encoders", "Right");*/

        pdp = new PowerDistributionPanel(); //TODO fix Shuffleboard readings
        pdp.clearStickyFaults();
        pdp.resetTotalEnergy();
    }

    @Override
    public void robotPeriodic() {
        //It's quiet here
    }

    @Override
    public void autonomousInit() {
        //leftDriveEncoder.reset();
        //rightDriveEncoder.reset();
    }

    @Override
    public void autonomousPeriodic() {
        //Nothing yet
    }

    @Override
    public void teleopInit() {
        //leftDriveEncoder.reset();
        //rightDriveEncoder.reset();
    }

    @Override
    public void teleopPeriodic() {
        robotDrive.arcadeDrive(-driveStick.getY(), driveWheel.getX());

        // AUTOMATIC SHIFTING
        /*if (leftDriveEncoder.getRate() > Parameters.UPPER_SHIFT_THRESHOLD
                && rightDriveEncoder.getRate() > Parameters.UPPER_SHIFT_THRESHOLD && inLowGear) {
            inLowGear = false;
            shifterSolenoid.set(DoubleSolenoid.Value.kReverse);
        } else if (leftDriveEncoder.getRate() < Parameters.LOWER_SHIFT_THRESHOLD
                && rightDriveEncoder.getRate() < Parameters.LOWER_SHIFT_THRESHOLD && !inLowGear) {
            inLowGear = true;
            shifterSolenoid.set(DoubleSolenoid.Value.kForward);
        }*/
    }

    @Override
    public void disabledInit() {
        robotDrive.stopMotor();
    }
}
