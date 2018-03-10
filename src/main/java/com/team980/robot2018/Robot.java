package com.team980.robot2018;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
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

    private PowerDistributionPanel pdp;
    private NetworkTable table;

    private Joystick driveStick;
    private Joystick driveWheel;
    private Joystick operatorController; //TODO: Wrapper class for gamepad that names all the buttons and sticks
    private Joystick prajBox;

    private DifferentialDrive robotDrive;

    private Encoder leftDriveEncoder;
    private Encoder rightDriveEncoder;

    private LiftSystem liftSystem;

    private WPI_TalonSRX climbMotor;

    private PigeonIMU imu;
    private double[] ypr;

    private DoubleSolenoid shifterSolenoid;
    private DoubleSolenoid clawSolenoid;

    private Rioduino coprocessor;

    private SendableChooser<Autonomous> autoChooser;
    private int turnAngle;
    private double scaleDistance;
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
        prajBox = new Joystick(3);

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

        liftSystem = new LiftSystem(table);

        climbMotor = new WPI_TalonSRX(Parameters.CLIMB_MOTOR_CAN_ID);
        climbMotor.setName("Climb Motor");

        imu = new PigeonIMU(Parameters.IMU_CAN_ID);
        PigeonGyro dashGyro = new PigeonGyro(imu);
        dashGyro.setName("Dashboard Gyro");
        ypr = new double[3];

        shifterSolenoid = new DoubleSolenoid(Parameters.PCM_CAN_ID, 0, 1);
        shifterSolenoid.setName("Pneumatics", "Shifter Solenoid");
        inLowGear = true;

        clawSolenoid = new DoubleSolenoid(Parameters.PCM_CAN_ID, 2, 3);
        clawSolenoid.setName("Pneumatics", "Claw Solenoid");

        coprocessor = new Rioduino();

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
        liftSystem.updateData();

        imu.getYawPitchRoll(ypr);
        coprocessor.updateData();

        if (autoChooser.getSelected() != null) {
            table.getEntry("Auto Selected").setString(autoChooser.getSelected().name());
        } else {
            table.getEntry("Auto Selected").setString("NULL");
        }

        table.getSubTable("IMU").getEntry("Yaw").setNumber(ypr[0]);
        table.getSubTable("IMU").getEntry("Pitch").setNumber(ypr[1]);
        table.getSubTable("IMU").getEntry("Roll").setNumber(ypr[2]);

        table.getSubTable("Coprocessor").getEntry("Vision Target Coord").setNumber(coprocessor.getVisionTargetCoord());
        table.getSubTable("Coprocessor").getEntry("Power Cube Width").setNumber(coprocessor.getPowerCubeWidth());
        table.getSubTable("Coprocessor").getEntry("Power Cube Height").setNumber(coprocessor.getPowerCubeHeight());
        table.getSubTable("Coprocessor").getEntry("Power Cube Coord").setNumber(coprocessor.getPowerCubeCoord());
        table.getSubTable("Coprocessor").getEntry("Sonar Distance").setNumber(coprocessor.getSonarDistance());
        table.getSubTable("Coprocessor").getEntry("Lidar Distance").setNumber(coprocessor.getLidarDistance());
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

        switch (autoChooser.getSelected()) {
            case LEFT_SIDE_CUBE_DROP:
                state = AutoState.STANDARD_START;
                turnAngle = Parameters.AUTO_LEFT_SIDE_TURN_ANGLE;
                break;
            case RIGHT_SIDE_CUBE_DROP:
                state = AutoState.STANDARD_START;
                turnAngle = Parameters.AUTO_RIGHT_SIDE_TURN_ANGLE;
                break;
            case CENTER_CUBE_DROP:
                state = AutoState.STANDARD_START;
                switch (MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR)) {
                    case LEFT: //Center, turn left to left plate
                        turnAngle = Parameters.AUTO_CENTER_LEFT_TURN_ANGLE;
                        break;
                    case RIGHT: //Center, turn right to right plate
                        turnAngle = Parameters.AUTO_CENTER_RIGHT_TURN_ANGLE;
                        break;
                    default: //We somehow don't know the angle...
                        turnAngle = Parameters.AUTO_CENTER_LEFT_TURN_ANGLE; //TODO set flag to never drop!
                        break;
                }
                break;
            case FAR_LEFT_GET_TO_SCALE:
                state = AutoState.GET_TO_SCALE;
                liftSystem.setPosition(LiftSystem.LiftPosition.SCALE);
                if (MatchData.getOwnedSide(MatchData.GameFeature.SCALE) == MatchData.OwnedSide.LEFT) {
                    scaleDistance = Parameters.AUTO_ALLIANCE_SCALE_DISTANCE;
                } else {
                    scaleDistance = Parameters.AUTO_OPPONENT_SCALE_DISTANCE;
                }
                break;
            case FAR_RIGHT_GET_TO_SCALE:
                state = AutoState.GET_TO_SCALE;
                liftSystem.setPosition(LiftSystem.LiftPosition.SCALE);
                if (MatchData.getOwnedSide(MatchData.GameFeature.SCALE) == MatchData.OwnedSide.RIGHT) {
                    scaleDistance = Parameters.AUTO_ALLIANCE_SCALE_DISTANCE;
                } else {
                    scaleDistance = Parameters.AUTO_OPPONENT_SCALE_DISTANCE;
                }
                break;
            default:
                state = AutoState.FINISHED;
                break;
        }
    }

    @Override
    public void autonomousPeriodic() {
        if (state != AutoState.FINISHED) {
            liftSystem.operateLift();
        }

        switch (state) {

            // SWITCH
            case STANDARD_START:
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

                if (Math.abs(turnAngle - ypr[0]) <= Parameters.AUTO_ANGULAR_DEADBAND) {
                    robotDrive.stopMotor();

                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();

                    state = AutoState.MOVE_TO_POSITION;
                } else {
                    robotDrive.arcadeDrive(0, -turnSpeed, false);
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
                int visionTargetOffset = coprocessor.getVisionTargetCoord() - 160 - 20; //off center
                turnSpeed = ((double) visionTargetOffset) / 160;

                int followDistance = coprocessor.getSonarDistance();
                double followSpeed = ((double) followDistance) / 20;
                if (Math.abs(followSpeed) > Parameters.AUTO_MAX_SPEED) {
                    followSpeed = Math.copySign(Parameters.AUTO_MAX_SPEED, followSpeed);
                }

                if (followDistance > 0 && followDistance < 12.5) { //Reached target... EXTERMINATE!
                    robotDrive.stopMotor();
                    state = AutoState.DEPOSIT_CUBE;
                } else if (coprocessor.getVisionTargetCoord() < 400) { //todo consistent
                    robotDrive.arcadeDrive(followSpeed, turnSpeed, false);
                } else {
                    robotDrive.stopMotor();
                    state = AutoState.FAILSAFE_TURN_TO_ZERO;
                }
                break;
            case DEPOSIT_CUBE:
                if (autoChooser.getSelected() == Autonomous.CENTER_CUBE_DROP
                        || (autoChooser.getSelected() == Autonomous.LEFT_SIDE_CUBE_DROP && MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR) == MatchData.OwnedSide.LEFT)
                        || (autoChooser.getSelected() == Autonomous.RIGHT_SIDE_CUBE_DROP && MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR) == MatchData.OwnedSide.RIGHT)) {
                    if (liftSystem.hasReachedPosition(LiftSystem.LiftPosition.SCALE)) {
                        robotDrive.arcadeDrive(0, 0);

                        clawSolenoid.set(DoubleSolenoid.Value.kForward); //open

                        leftDriveEncoder.reset();
                        rightDriveEncoder.reset();

                        state = AutoState.BACK_UP_FROM_TARGET;
                    }
                } else {
                    //Wrong side! Stop here
                    state = AutoState.FINISHED;
                }
                break;
            case BACK_UP_FROM_TARGET:
                if (leftDriveEncoder.getDistance() < -Parameters.AUTO_STARTING_DISTANCE
                        || rightDriveEncoder.getDistance() < -Parameters.AUTO_STARTING_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.TURN_AWAY_FROM_TARGET;
                } else {
                    robotDrive.arcadeDrive(-Parameters.AUTO_MAX_SPEED, 0, false);
                }
                break;
            case TURN_AWAY_FROM_TARGET:
                turnSpeed = (Math.copySign(90, turnAngle) - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(Math.copySign(90, turnAngle) - ypr[0]) > Parameters.AUTO_ANGULAR_DEADBAND) { //todo consistent
                    robotDrive.arcadeDrive(0, -turnSpeed, false);
                } else {
                    robotDrive.stopMotor();
                    liftSystem.setPosition(LiftSystem.LiftPosition.BOTTOM);

                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();

                    state = AutoState.MOVE_PAST_TARGET;
                }
                break;
            case MOVE_PAST_TARGET:
                if (leftDriveEncoder.getDistance() > Parameters.AUTO_PAST_TARGET_DISTANCE
                        || rightDriveEncoder.getDistance() > Parameters.AUTO_PAST_TARGET_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.TURN_TO_ALLIANCE;
                } else {
                    robotDrive.arcadeDrive(Parameters.AUTO_MAX_SPEED, 0, false);
                }
                break;
            case TURN_TO_ALLIANCE:
                turnSpeed = (Math.copySign(180, turnAngle) - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(Math.copySign(180, turnAngle) - ypr[0]) <= Parameters.AUTO_ANGULAR_DEADBAND) {
                    robotDrive.stopMotor();

                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();

                    state = AutoState.MOVE_PAST_SWITCH;
                } else {
                    robotDrive.arcadeDrive(0, -turnSpeed, false);
                }
                break;
            case MOVE_PAST_SWITCH:
                if (leftDriveEncoder.getDistance() < -Parameters.AUTO_PAST_SWITCH_DISTANCE
                        || rightDriveEncoder.getDistance() < -Parameters.AUTO_PAST_SWITCH_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.PAC_MAN_MODE;
                } else {
                    robotDrive.arcadeDrive(-Parameters.AUTO_MAX_SPEED, 0, false);
                }
                break;
            case PAC_MAN_MODE:
                int powerCubeOffset = coprocessor.getPowerCubeCoord() - 160 - 20; //off center
                turnSpeed = ((double) powerCubeOffset) / 160;

                followDistance = coprocessor.getSonarDistance();
                followSpeed = ((double) followDistance) / 20;
                if (Math.abs(followSpeed) > 0.6) {
                    followSpeed = Math.copySign(0.6, followSpeed);
                }

                System.out.println(coprocessor.getSonarDistance());
                if (coprocessor.getSonarDistance() < 15) { //Cube in mouth... eat it! - TODO consistent
                    robotDrive.stopMotor();
                    state = AutoState.EAT_CUBE;
                } else if (coprocessor.getPowerCubeCoord() < 400) {
                    robotDrive.arcadeDrive(followSpeed, turnSpeed, false);
                } else {
                    robotDrive.stopMotor();
                    state = AutoState.FINISHED;
                }
                break;
            case EAT_CUBE:
                robotDrive.arcadeDrive(0, 0);
                clawSolenoid.set(DoubleSolenoid.Value.kReverse); //eat the cube
                state = AutoState.FINISHED;
                break;

            // SWITCH FAILSAFE
            case FAILSAFE_TURN_TO_ZERO:
                turnSpeed = (0 - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(0 - ypr[0]) <= Parameters.AUTO_ANGULAR_DEADBAND) {
                    robotDrive.stopMotor();

                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();

                    state = AutoState.FAILSAFE_DRIVE_FORWARD;
                } else {
                    robotDrive.arcadeDrive(0, -turnSpeed, false);
                }
                break;
            case FAILSAFE_DRIVE_FORWARD:
                if (leftDriveEncoder.getDistance() > Parameters.AUTO_BACKUP_DISTANCE
                        || rightDriveEncoder.getDistance() > Parameters.AUTO_BACKUP_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.FINISHED;
                } else {
                    robotDrive.arcadeDrive(Parameters.AUTO_MAX_SPEED, 0, false);
                }
                break;

            // SCALE
            case GET_TO_SCALE:
                if (leftDriveEncoder.getDistance() < -scaleDistance
                        || rightDriveEncoder.getDistance() < -scaleDistance) {
                    robotDrive.stopMotor();
                    state = AutoState.FINISHED;
                } else {
                    robotDrive.arcadeDrive(-Parameters.AUTO_MAX_SPEED, 0, false);
                }
                break;

            // FINISHED
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
    }

    @Override
    public void teleopPeriodic() {
        if (Math.abs(ypr[2]) >= 30) { // TIPPING PROTECTION - OVERRIDE driver input
            robotDrive.arcadeDrive(Math.copySign(0.4, ypr[2]), 0);
        } else {
            robotDrive.arcadeDrive(-driveStick.getY(), driveWheel.getX());
        }

        liftSystem.operateLift(operatorController);

        // CLIMBER CONTROL
        if (prajBox.getRawButton(0)) {
            if (prajBox.getRawButton(4)) { //Only allow backdrive if an extra button is being held
                climbMotor.set(operatorController.getRawAxis(5));
            } else {
                climbMotor.set(Math.abs(operatorController.getRawAxis(5)));
            }
        } else {
            climbMotor.set(0);
        }

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

        if (operatorController.getRawButton(6)) {
            liftSystem.setPosition(LiftSystem.LiftPosition.SCALE);
        }

        if (operatorController.getRawButton(1)) {
            liftSystem.setPosition(LiftSystem.LiftPosition.SWITCH);
        }

        if (operatorController.getRawButton(5)) {
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
        if (liftSystem.hasReachedPosition(LiftSystem.LiftPosition.SCALE) && !inLowGear) { //hack shift limit
            inLowGear = true;
            shifterSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            if (Math.abs(leftDriveEncoder.getRate()) > Parameters.UPPER_SHIFT_THRESHOLD
                    && Math.abs(rightDriveEncoder.getRate()) > Parameters.UPPER_SHIFT_THRESHOLD && inLowGear) {
                inLowGear = false;
                shifterSolenoid.set(DoubleSolenoid.Value.kReverse);
            } else if (Math.abs(leftDriveEncoder.getRate()) < Parameters.LOWER_SHIFT_THRESHOLD
                    && Math.abs(rightDriveEncoder.getRate()) < Parameters.LOWER_SHIFT_THRESHOLD && !inLowGear) {
                inLowGear = true;
                shifterSolenoid.set(DoubleSolenoid.Value.kForward);
            }
        }

        //New Power Cube Eating Mode - PAC MAN!
        if (pacManMode) {
            int visionTargetOffset = coprocessor.getPowerCubeCoord() - 160 - 20; //off center
            double turnSpeed = ((double) visionTargetOffset) / 160;

            int followDistance = coprocessor.getSonarDistance();
            double followSpeed = ((double) followDistance) / 20;
            if (Math.abs(followSpeed) > 0.6) {
                followSpeed = Math.copySign(0.6, followSpeed);
            }

            System.out.println(coprocessor.getSonarDistance());
            if (coprocessor.getSonarDistance() < 15) { //Cube in mouth... eat it!
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

        climbMotor.disable();

        pacManMode = false;
    }

    public enum Autonomous {
        DISABLED,

        LEFT_SIDE_CUBE_DROP,
        RIGHT_SIDE_CUBE_DROP,
        CENTER_CUBE_DROP,

        FAR_LEFT_GET_TO_SCALE,
        FAR_RIGHT_GET_TO_SCALE
    }

    public enum AutoState {
        STANDARD_START,
        TURN_TO_ANGLE,
        MOVE_TO_POSITION,
        DALEK_MODE,
        DEPOSIT_CUBE,
        BACK_UP_FROM_TARGET,
        TURN_AWAY_FROM_TARGET,
        MOVE_PAST_TARGET,
        TURN_TO_ALLIANCE,
        MOVE_PAST_SWITCH,
        PAC_MAN_MODE,
        EAT_CUBE,

        FAILSAFE_TURN_TO_ZERO,
        FAILSAFE_DRIVE_FORWARD,

        GET_TO_SCALE,

        FINISHED
    }
}
