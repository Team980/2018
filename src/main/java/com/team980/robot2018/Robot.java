package com.team980.robot2018;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import openrio.powerup.MatchData;

public class Robot extends IterativeRobot { //TODO test TimedRobot - exact 20ms vs about 20ms

    private Joystick driveStick;
    private Joystick driveWheel;
    private Joystick operatorBox;
    private Joystick gameController;

    private Spark leftDrive; //2017 robot uses Sparks
    private Spark rightDrive; //We'll be testing on it until we get our drive base - with Victor SPXes
    private DifferentialDrive robotDrive;

    private Encoder leftDriveEncoder;
    private Encoder rightDriveEncoder;

    private PigeonIMU imu;
    private double[] ypr;

    private Solenoid shifterSolenoid;
    private boolean inLowGear;

    private Rioduino coprocessor;

    private NetworkTable table;

    private SendableChooser<Autonomous> autoChooser;
    private int turnAngle;
    private AutoState state;

    private boolean dalekMode = false;

    @Override
    public void robotInit() {
        driveStick = new Joystick(Parameters.DRIVE_STICK_JS_ID);
        driveWheel = new Joystick(Parameters.DRIVE_WHEEL_JS_ID);
        operatorBox = new Joystick(Parameters.OPERATOR_BOX_JS_ID);
        gameController = new Joystick(Parameters.GAME_CONTROLLER_JS_ID);

        leftDrive = new Spark(Parameters.LEFT_DRIVE_PWM_CHANNEL);
        rightDrive = new Spark(Parameters.RIGHT_DRIVE_PWM_CHANNEL);
        robotDrive = new DifferentialDrive(leftDrive, rightDrive);

        leftDriveEncoder = new Encoder(Parameters.LEFT_ENCODER_DIO_CHANNEL_A, Parameters.LEFT_ENCODER_DIO_CHANNEL_B, Parameters.INVERT_LEFT_ENCODER, CounterBase.EncodingType.k4X);
        leftDriveEncoder.setDistancePerPulse((2 * (Constants.PI) * (Constants.wheelRadius / 12)) / (Constants.encoderPulsesPerRevolution));
        leftDriveEncoder.setPIDSourceType(PIDSourceType.kRate);

        rightDriveEncoder = new Encoder(Parameters.RIGHT_ENCODER_DIO_CHANNEL_A, Parameters.RIGHT_ENCODER_DIO_CHANNEL_B, Parameters.INVERT_RIGHT_ENCODER, CounterBase.EncodingType.k4X);
        rightDriveEncoder.setDistancePerPulse((2 * (Constants.PI) * (Constants.wheelRadius / 12)) / (Constants.encoderPulsesPerRevolution));
        rightDriveEncoder.setPIDSourceType(PIDSourceType.kRate);

        imu = new PigeonIMU(Parameters.IMU_CAN_ID);
        ypr = new double[3];

        shifterSolenoid = new Solenoid(Parameters.PCM_CAN_ID, Parameters.SHIFTER_SOLENOID_CHANNEL);
        inLowGear = true;

        coprocessor = new Rioduino();

        table = NetworkTableInstance.getDefault().getTable("ThunderBots");

        autoChooser = new SendableChooser<>();
        autoChooser.addObject("Disabled", Autonomous.DISABLED);
        autoChooser.addObject("Left Side - Cube Drop", Autonomous.LEFT_SIDE_CUBE_DROP);
        autoChooser.addObject("Right Side - Cube Drop", Autonomous.RIGHT_SIDE_CUBE_DROP);
        autoChooser.addObject("Center - Cube Drop", Autonomous.CENTER_CUBE_DROP);
        autoChooser.addObject("Far Left - Get To Scale", Autonomous.FAR_LEFT_GET_TO_SCALE);
        SmartDashboard.putData(autoChooser); // TODO this is broken

        table.getEntry("Autonomous Mode").setString("Off");

        PowerDistributionPanel pdp = new PowerDistributionPanel(); //TODO voltage safety
    }

    @Override
    public void robotPeriodic() {
        imu.getYawPitchRoll(ypr);
        coprocessor.updateData();

        table.getSubTable("IMU").getEntry("Yaw").setNumber(ypr[0]);
        table.getSubTable("IMU").getEntry("Pitch").setNumber(ypr[1]);
        table.getSubTable("IMU").getEntry("Roll").setNumber(ypr[2]);

        table.getSubTable("Encoder Distance").getEntry("Left").setNumber(leftDriveEncoder.getDistance());
        table.getSubTable("Encoder Distance").getEntry("Right").setNumber(rightDriveEncoder.getDistance());

        table.getSubTable("Encoder Rate").getEntry("Left").setNumber(leftDriveEncoder.getRate());
        table.getSubTable("Encoder Rate").getEntry("Right").setNumber(rightDriveEncoder.getRate());

        table.getSubTable("Coprocessor").getEntry("Vision Target Coord").setNumber(coprocessor.getVisionTargetCoord());
        table.getSubTable("Coprocessor").getEntry("Ranged Distance").setNumber(coprocessor.getRangedDistance());
    }

    @Override
    public void autonomousInit() {
        leftDriveEncoder.reset();
        rightDriveEncoder.reset();

        imu.setYaw(0, 0); //reset

        shifterSolenoid.set(true); //low

        switch (Autonomous.CENTER_CUBE_DROP) { //TODO autoChooser.getSelected()
            case LEFT_SIDE_CUBE_DROP:
                turnAngle = Parameters.AUTO_LEFT_SIDE_TURN_ANGLE;
                break;
            case RIGHT_SIDE_CUBE_DROP:
                turnAngle = Parameters.AUTO_RIGHT_SIDE_TURN_ANGLE;
                break;
            case CENTER_CUBE_DROP:
                switch (MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR)) {
                    case LEFT: //Center, turn left to left plate
                        turnAngle = Parameters.AUTO_CENTER_LEFT_TURN_ANGLE;
                        break;
                    case RIGHT: //Center, turn right to right plate
                        turnAngle = Parameters.AUTO_CENTER_RIGHT_TURN_ANGLE;
                        break;
                    default: //We somehow don't know the angle...
                        turnAngle = Parameters.AUTO_CENTER_RIGHT_TURN_ANGLE; //TODO set flag to never drop!
                        break;
                }
                break;
            case FAR_LEFT_GET_TO_SCALE:
                //TODO
                break;
        }

        state = AutoState.START;
    }

    @Override
    public void autonomousPeriodic() {
        switch (state) {
            case START: //TODO lift cube at same time
                if (leftDriveEncoder.getDistance() > Parameters.AUTO_STARTING_DISTANCE
                        || rightDriveEncoder.getDistance() > Parameters.AUTO_STARTING_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.TURN_TO_ANGLE;
                } else {
                    robotDrive.arcadeDrive(Parameters.AUTO_MAX_SPEED, 0, false);
                }
                break;
            case TURN_TO_ANGLE:
                double turnSpeed = (turnAngle - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(turnAngle - ypr[0]) > Parameters.AUTO_ANGULAR_DEADBAND) { //todo consistent
                    robotDrive.arcadeDrive(0, -turnSpeed, false);
                } else {
                    robotDrive.stopMotor();
                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();
                    state = AutoState.MOVE_TO_POSITION;
                }
                break;
            case MOVE_TO_POSITION:
                if (leftDriveEncoder.getDistance() > Parameters.AUTO_POSITIONING_DISTANCE
                        || rightDriveEncoder.getDistance() > Parameters.AUTO_POSITIONING_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.DALEK_MODE;
                } else {
                    robotDrive.arcadeDrive(Parameters.AUTO_MAX_SPEED, 0, false);
                }
                break;
            case DALEK_MODE: // SEEK - LOCATE - DESTROY!
                int visionTargetOffset = coprocessor.getVisionTargetCoord() - 160;
                turnSpeed = ((double) visionTargetOffset) / 160;

                int followDistance = coprocessor.getRangedDistance();
                double followSpeed = ((double) followDistance) / 1500;
                if (Math.abs(followSpeed) > Parameters.AUTO_MAX_SPEED) {
                    followSpeed = Math.copySign(Parameters.AUTO_MAX_SPEED, followSpeed);
                }

                if (followDistance < 320) { //Reached target... EXTERMINATE!
                    robotDrive.stopMotor();
                    state = AutoState.DEPOSIT_CUBE;
                } else if (coprocessor.getVisionTargetCoord() > 0 && coprocessor.getVisionTargetCoord() < 400) { //todo consistent
                    robotDrive.arcadeDrive(followSpeed, turnSpeed, false);
                } else {
                    robotDrive.stopMotor();
                    state = AutoState.BACKUP_TURN_TO_ZERO;
                }
                break;
            case DEPOSIT_CUBE: //TODO
                state = AutoState.FINISHED;
                break;
            case BACKUP_TURN_TO_ZERO:
                turnSpeed = (0 - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(0 - ypr[0]) > Parameters.AUTO_ANGULAR_DEADBAND) { //todo consistent
                    robotDrive.arcadeDrive(0, -turnSpeed, false);
                } else {
                    robotDrive.stopMotor();
                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();
                    state = AutoState.BACKUP_DRIVE_FORWARD;
                }
                break;
            case BACKUP_DRIVE_FORWARD:
                if (leftDriveEncoder.getDistance() > Parameters.AUTO_BACKUP_DISTANCE
                        || rightDriveEncoder.getDistance() > Parameters.AUTO_BACKUP_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.FINISHED;
                } else {
                    robotDrive.arcadeDrive(Parameters.AUTO_MAX_SPEED, 0, false);
                }
                break;
            case FINISHED:
                //Now we're done!
                break;
        }

        table.getEntry("Autonomous Mode").setString(state.name());
    }

    @Override
    public void teleopInit() {
        leftDriveEncoder.reset();
        rightDriveEncoder.reset();

        shifterSolenoid.set(true); //low
    }

    @Override
    public void teleopPeriodic() {

        switch (Parameters.CONTROL_MODE) {
            case COMPETITION_DRIVER_STATION:
                teleopOperatorControls(operatorBox);
                robotDrive.arcadeDrive(driveStick.getY(), driveWheel.getX());
                break;
            case SINGLE_JOYSTICK:
                teleopOperatorControls(driveStick);
                robotDrive.arcadeDrive(-driveStick.getY(), driveStick.getX()); //Z axis is improperly calibrated >:(
                break;
            case GAME_CONTROLLER:
                teleopOperatorControls(gameController);
                robotDrive.arcadeDrive((gameController.getRawAxis(3) - gameController.getRawAxis(2)), -gameController.getRawAxis(0));
                break;
        }

        // AUTOMATIC SHIFTING - TODO FIX LEFT ENCODER
        if (/*leftDriveEncoder.getRate() > Parameters.UPPER_SHIFT_THRESHOLD
                &&*/ rightDriveEncoder.getRate() > Parameters.UPPER_SHIFT_THRESHOLD
                && inLowGear) {
            inLowGear = false;
            shifterSolenoid.set(false);
        } else if (/*leftDriveEncoder.getRate() < Parameters.LOWER_SHIFT_THRESHOLD
                &&*/ rightDriveEncoder.getRate() < Parameters.LOWER_SHIFT_THRESHOLD
                && !inLowGear) {
            inLowGear = true;
            shifterSolenoid.set(true);
        }

        if (dalekMode) { //TODO remove before competition?
            int visionTargetOffset = coprocessor.getVisionTargetCoord() - 160;
            double turnSpeed = ((double) visionTargetOffset) / 160;

            int followDistance = coprocessor.getRangedDistance();
            double followSpeed = ((double) followDistance) / 1500;
            if (Math.abs(followSpeed) > 0.6) {
                followSpeed = Math.copySign(0.6, followSpeed);
            }

            if (/*Math.abs(visionTargetOffset) > 20 && Math.abs(followDistance) > 300 &&*/
                    coprocessor.getVisionTargetCoord() > 0 && coprocessor.getVisionTargetCoord() < 400) {
                System.out.println("Following at " + followSpeed + "; turning at " + turnSpeed);
                robotDrive.arcadeDrive(followSpeed, turnSpeed, false);
            } else {
                System.out.println("No target found, Stopping");
                robotDrive.stopMotor();
                dalekMode = false;
            }
        }
    }


    /**
     * Teleop button controls that don't change based on the control mode go here.
     * The button assignments will remain constant.
     */
    private void teleopOperatorControls(Joystick js) {
        if (js.getRawButton(1)) {
            System.out.println("Entering Dalek mode");
            System.out.println("SEEK - LOCATE - DESTROY");
            dalekMode = true;
        }

        if (js.getRawButton(2)) {
            System.out.println("Leaving Dalek mode");
            dalekMode = false;
        }

        /*if (js.getRawButtonPressed(3)) {
            shifterSolenoid.set(true); //low
            inLowGear = true;
        }

        if (js.getRawButtonPressed(4)) {
            shifterSolenoid.set(false); //high
            inLowGear = false;
        }*/
    }

    @Override
    public void disabledInit() {
        robotDrive.stopMotor();

        dalekMode = false;
    }

    public enum Autonomous {
        DISABLED,
        LEFT_SIDE_CUBE_DROP,
        RIGHT_SIDE_CUBE_DROP,
        CENTER_CUBE_DROP,
        FAR_LEFT_GET_TO_SCALE
    }

    public enum AutoState {
        START,
        TURN_TO_ANGLE,
        MOVE_TO_POSITION,
        DALEK_MODE,
        BACKUP_TURN_TO_ZERO,
        BACKUP_DRIVE_FORWARD,
        DEPOSIT_CUBE,
        FINISHED
    }
}
