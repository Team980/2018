package com.team980.robot2018;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team980.robot2018.sensors.LaserRangefinder;
import com.team980.robot2018.sensors.Rioduino;
import com.team980.robot2018.subsystems.LiftSystem;
import com.team980.robot2018.util.PigeonGyro;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import openrio.powerup.MatchData;

public class Robot extends TimedRobot {
    //TODO reorganize all these variables

    private PowerDistributionPanel pdp;
    private NetworkTable table;

    private Joystick driveStick;
    private Joystick driveWheel;
    private Joystick operatorController; //TODO: Wrapper class for gamepad that names all the buttons and sticks

    private DifferentialDrive robotDrive;

    private Encoder leftDriveEncoder;
    private Encoder rightDriveEncoder;

    private LiftSystem liftSystem;
    private LaserRangefinder rangefinder;

    private PigeonIMU imu;
    private double[] ypr;

    private DoubleSolenoid shifterSolenoid;
    private DoubleSolenoid clawSolenoid;

    private Rioduino coprocessor;

    private Relay dalekEye;

    private SendableChooser<Autonomous> autoChooser;
    private int turnAngle;
    private AutoState state;

    private boolean inLowGear = true;
    private boolean pacManMode = false;

    @Override
    public void robotInit() {
        pdp = new PowerDistributionPanel();
        pdp.clearStickyFaults();
        pdp.resetTotalEnergy();

        table = NetworkTableInstance.getDefault().getTable("ThunderBots");

        driveStick = new Joystick(Parameters.DRIVE_STICK_JS_ID);
        driveWheel = new Joystick(Parameters.DRIVE_WHEEL_JS_ID);
        operatorController = new Joystick(Parameters.OPERATOR_CONTROLLER_JS_ID);

        SpeedControllerGroup leftDrive = new SpeedControllerGroup(new WPI_TalonSRX(Parameters.LEFT_FRONT_DRIVE_CAN_ID),
                new WPI_TalonSRX(Parameters.LEFT_BACK_DRIVE_CAN_ID));
        leftDrive.setInverted(true);

        SpeedControllerGroup rightDrive = new SpeedControllerGroup(new WPI_TalonSRX(Parameters.RIGHT_FRONT_DRIVE_CAN_ID),
                new WPI_TalonSRX(Parameters.RIGHT_BACK_DRIVE_CAN_ID));
        rightDrive.setInverted(true);

        robotDrive = new DifferentialDrive(leftDrive, rightDrive);
        robotDrive.setName("Robot Drive");

        leftDriveEncoder = new Encoder(Parameters.LEFT_DRIVE_ENCODER_DIO_CHANNEL_A, Parameters.LEFT_DRIVE_ENCODER_DIO_CHANNEL_B, Parameters.INVERT_LEFT_DRIVE_ENCODER, CounterBase.EncodingType.k4X);
        leftDriveEncoder.setDistancePerPulse((2 * (Constants.PI) * (Constants.DRIVE_WHEEL_RADIUS / 12)) / (Constants.DRIVE_ENCODER_PULSES_PER_REVOLUTION));
        leftDriveEncoder.setName("Drive Encoders", "Left");

        rightDriveEncoder = new Encoder(Parameters.RIGHT_DRIVE_ENCODER_DIO_CHANNEL_A, Parameters.RIGHT_DRIVE_ENCODER_DIO_CHANNEL_B, Parameters.INVERT_RIGHT_DRIVE_ENCODER, CounterBase.EncodingType.k4X);
        rightDriveEncoder.setDistancePerPulse((2 * (Constants.PI) * (Constants.DRIVE_WHEEL_RADIUS / 12)) / (Constants.DRIVE_ENCODER_PULSES_PER_REVOLUTION));
        rightDriveEncoder.setName("Drive Encoders", "Right");

        liftSystem = new LiftSystem(pdp, table);

        rangefinder = new LaserRangefinder();

        imu = new PigeonIMU(Parameters.IMU_CAN_ID);
        PigeonGyro dashGyro = new PigeonGyro(imu);
        dashGyro.setName("Dashboard Gyro");
        ypr = new double[3];

        shifterSolenoid = new DoubleSolenoid(Parameters.PCM_CAN_ID, 0, 1);//TODO go back to singular (Parameters.PCM_CAN_ID, Parameters.SHIFTER_SOLENOID_CHANNEL);
        shifterSolenoid.setName("Pneumatics", "Shifter Solenoid");
        inLowGear = true;

        clawSolenoid = new DoubleSolenoid(Parameters.PCM_CAN_ID, 2, 3);
        clawSolenoid.setName("Pneumatics", "Claw Solenoid");

        coprocessor = new Rioduino();

        dalekEye = new Relay(Parameters.DALEK_EYE_RELAY_CHANNEL);
        dalekEye.setName("Dalek Eye");

        autoChooser = new SendableChooser<>();
        autoChooser.addDefault("Disabled", Autonomous.DISABLED);
        autoChooser.addObject("Left Side - Cube Drop", Autonomous.LEFT_SIDE_CUBE_DROP);
        autoChooser.addObject("Right Side - Cube Drop", Autonomous.RIGHT_SIDE_CUBE_DROP);
        autoChooser.addObject("Center - Cube Drop", Autonomous.CENTER_CUBE_DROP);
        autoChooser.addObject("Far Left - Get To Scale", Autonomous.FAR_LEFT_GET_TO_SCALE);
        autoChooser.setName("Autonomous Chooser");
        LiveWindow.add(autoChooser); //This actually works

        table.getEntry("Autonomous State").setString("");
    }

    @Override
    public void robotPeriodic() {
        imu.getYawPitchRoll(ypr);

        liftSystem.updateData();

        coprocessor.updateData();
        rangefinder.updateData();

        if (autoChooser.getSelected() != null) {
            table.getEntry("Auto Selected").setString(autoChooser.getSelected().name());
        } else {
            table.getEntry("Auto Selected").setString("NULL");
        }

        table.getSubTable("IMU").getEntry("Yaw").setNumber(ypr[0]);
        table.getSubTable("IMU").getEntry("Pitch").setNumber(ypr[1]);
        table.getSubTable("IMU").getEntry("Roll").setNumber(ypr[2]);

        table.getSubTable("Lift System").getEntry("Laser Rangefinder").setNumber(rangefinder.getDistance());

        table.getSubTable("Coprocessor").getEntry("Vision Target Coord").setNumber(coprocessor.getVisionTargetCoord());
        table.getSubTable("Coprocessor").getEntry("Power Cube Width").setNumber(coprocessor.getPowerCubeWidth());
        table.getSubTable("Coprocessor").getEntry("Power Cube Height").setNumber(coprocessor.getPowerCubeHeight());
        table.getSubTable("Coprocessor").getEntry("Power Cube Coord").setNumber(coprocessor.getPowerCubeCoord());
        table.getSubTable("Coprocessor").getEntry("Ranged Distance").setNumber(coprocessor.getRangedDistance());
    }

    @Override
    public void autonomousInit() {
        leftDriveEncoder.reset();
        rightDriveEncoder.reset();

        liftSystem.resetEncoder();
        liftSystem.setPosition(LiftSystem.LiftPosition.SCALE);

        imu.setYaw(0, 0);

        shifterSolenoid.set(DoubleSolenoid.Value.kForward); //low
        clawSolenoid.set(DoubleSolenoid.Value.kReverse); //closed

        dalekEye.set(Relay.Value.kForward);

        switch (autoChooser.getSelected()) {
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

        if (autoChooser.getSelected() == Autonomous.DISABLED) {
            state = AutoState.FINISHED;
        } else {
            state = AutoState.START;
        }
    }

    @Override
    public void autonomousPeriodic() { //TODO we always need to be telling arcadeDrive something - work with class extraction
        if (state != AutoState.FINISHED) {
            liftSystem.operateLift();
        }

        switch (state) {
            case START:
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
                int visionTargetOffset = coprocessor.getVisionTargetCoord() - 160 - 30; //off center
                turnSpeed = ((double) visionTargetOffset) / 160;

                int followDistance = coprocessor.getRangedDistance();
                double followSpeed = ((double) followDistance) / 500;
                if (Math.abs(followSpeed) > Parameters.AUTO_MAX_SPEED) {
                    followSpeed = Math.copySign(Parameters.AUTO_MAX_SPEED, followSpeed);
                }

                if (followDistance > 0 && followDistance < 320) { //Reached target... EXTERMINATE!
                    robotDrive.stopMotor();
                    state = AutoState.DEPOSIT_CUBE;
                } else if (coprocessor.getVisionTargetCoord() < 400) { //todo consistent
                    robotDrive.arcadeDrive(followSpeed, turnSpeed, false);
                } else {
                    robotDrive.stopMotor();
                    state = AutoState.BACKUP_TURN_TO_ZERO;
                }
                break;
            case DEPOSIT_CUBE:
                if (liftSystem.hasReachedPosition(LiftSystem.LiftPosition.SCALE)) {
                    clawSolenoid.set(DoubleSolenoid.Value.kForward); //open

                    state = AutoState.FINISHED;
                }
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

        table.getEntry("Autonomous State").setString(state.name());
    }

    @Override
    public void teleopInit() {
        leftDriveEncoder.reset();
        rightDriveEncoder.reset();

        imu.setYaw(0, 0);

        shifterSolenoid.set(DoubleSolenoid.Value.kForward); //low
        clawSolenoid.set(DoubleSolenoid.Value.kReverse); //closed

        dalekEye.set(Relay.Value.kOff);
    }

    @Override
    public void teleopPeriodic() {
        robotDrive.arcadeDrive(-driveStick.getY(), driveWheel.getX());
        liftSystem.operateLift(operatorController);

        if (operatorController.getRawButton(2)) {
            System.out.println("Entering PAC MAN mode");
            System.out.println("wakawakawakawakawaka");
            liftSystem.setPosition(LiftSystem.LiftPosition.BOTTOM);
            clawSolenoid.set(DoubleSolenoid.Value.kForward); //open
            pacManMode = true;
        }

        if (operatorController.getRawButton(4)) {
            System.out.println("Leaving PAC MAN mode");
            pacManMode = false;
        }

        if (operatorController.getRawButtonPressed(7)) {
            shifterSolenoid.set(DoubleSolenoid.Value.kForward); //low
            inLowGear = true;
        }

        if (operatorController.getRawButtonPressed(8)) {
            shifterSolenoid.set(DoubleSolenoid.Value.kReverse); //high
            inLowGear = false;
        }

        if (operatorController.getRawButton(5)) {
            liftSystem.setPosition(LiftSystem.LiftPosition.SCALE);
        }

        if (operatorController.getRawButton(1)) {
            liftSystem.setPosition(LiftSystem.LiftPosition.SWITCH);
        }

        if (operatorController.getRawButton(6)) {
            liftSystem.setPosition(LiftSystem.LiftPosition.BOTTOM);
        }

        if (operatorController.getRawAxis(2) > 0.9) {
            clawSolenoid.set(DoubleSolenoid.Value.kForward); //open
        }

        if (operatorController.getRawAxis(3) > 0.9) {
            clawSolenoid.set(DoubleSolenoid.Value.kReverse); //closed
        }

        if (operatorController.getRawButtonPressed(9)) {
            imu.setYaw(0, 0);
        }

        if (operatorController.getRawButtonPressed(10)) {
            liftSystem.resetEncoder();
        }

        // AUTOMATIC SHIFTING
        if (leftDriveEncoder.getRate() > Parameters.UPPER_SHIFT_THRESHOLD
                && rightDriveEncoder.getRate() > Parameters.UPPER_SHIFT_THRESHOLD && inLowGear) {
            inLowGear = false;
            shifterSolenoid.set(DoubleSolenoid.Value.kReverse);
        } else if (leftDriveEncoder.getRate() < Parameters.LOWER_SHIFT_THRESHOLD
                && rightDriveEncoder.getRate() < Parameters.LOWER_SHIFT_THRESHOLD && !inLowGear) {
            inLowGear = true;
            shifterSolenoid.set(DoubleSolenoid.Value.kForward);
        }

        //New Power Cube Eating Mode - PAC MAN!
        if (pacManMode) {
            int visionTargetOffset = coprocessor.getPowerCubeCoord() - 160 - 30; //off center
            double turnSpeed = ((double) visionTargetOffset) / 160;

            int followDistance = coprocessor.getRangedDistance();
            double followSpeed = ((double) followDistance) / 500;
            if (Math.abs(followSpeed) > 0.6) {
                followSpeed = Math.copySign(0.6, followSpeed);
            }

            System.out.println(coprocessor.getRangedDistance());
            if (coprocessor.getRangedDistance() < 350) { //Cube in mouth... eat it!
                System.out.println("cube found - nomnomnom");
                robotDrive.stopMotor();
                clawSolenoid.set(DoubleSolenoid.Value.kReverse); //eat the cube
                pacManMode = false;
            } else if (coprocessor.getPowerCubeCoord() < 400) {
                System.out.println("Following at " + followSpeed + "; turning at " + turnSpeed);
                robotDrive.arcadeDrive(followSpeed, turnSpeed, false);
            } else {
                System.out.println("No target found, Stopping");
                robotDrive.stopMotor();
                pacManMode = false;
            }
        }
    }

    @Override
    public void disabledInit() {
        robotDrive.stopMotor();
        liftSystem.disable();

        pacManMode = false;
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
