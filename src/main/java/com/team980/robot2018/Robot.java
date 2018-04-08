package com.team980.robot2018;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
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

    private PIDController leftDriveController;
    private PIDController rightDriveController;

    private LiftSystem liftSystem;

    private WPI_TalonSRX climbMotor;

    private PigeonIMU imu;
    private double[] ypr;

    private Solenoid shifterSolenoid;
    private Solenoid clawSolenoid;

    private Rioduino coprocessor;

    private SendableChooser<Autonomous> autoChooser;
    private int turnAngle;
    private int loopCounter;
    private AutoState state;

    private boolean inLowGear = true;
    private boolean pacManMode = false;
    private boolean isTipping = false;

    @Override
    public void robotInit() {
        pdp = new PowerDistributionPanel();
        pdp.clearStickyFaults();
        pdp.resetTotalEnergy();

        table = NetworkTableInstance.getDefault().getTable("ThunderBots");

        driveStick = new Joystick(Parameters.DRIVE_STICK_JS_ID);
        driveWheel = new Joystick(Parameters.DRIVE_WHEEL_JS_ID);
        operatorController = new Joystick(Parameters.OPERATOR_CONTROLLER_JS_ID);
        prajBox = new Joystick(Parameters.PRAJ_BOX_JS_ID);

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

        leftDriveEncoder = new Encoder(Parameters.LEFT_DRIVE_ENCODER_DIO_CHANNEL_A, Parameters.LEFT_DRIVE_ENCODER_DIO_CHANNEL_B, Parameters.INVERT_LEFT_DRIVE_ENCODER, CounterBase.EncodingType.k4X);
        leftDriveEncoder.setDistancePerPulse((2 * (Constants.PI) * (Constants.DRIVE_WHEEL_RADIUS / 12)) / (Constants.DRIVE_ENCODER_PULSES_PER_REVOLUTION * Constants.DRIVE_SYSTEM_GEAR_RATIO));
        leftDriveEncoder.setPIDSourceType(PIDSourceType.kRate);
        leftDriveEncoder.setName("Drive Encoders", "Left");

        rightDriveEncoder = new Encoder(Parameters.RIGHT_DRIVE_ENCODER_DIO_CHANNEL_A, Parameters.RIGHT_DRIVE_ENCODER_DIO_CHANNEL_B, Parameters.INVERT_RIGHT_DRIVE_ENCODER, CounterBase.EncodingType.k4X);
        rightDriveEncoder.setDistancePerPulse((2 * (Constants.PI) * (Constants.DRIVE_WHEEL_RADIUS / 12)) / (Constants.DRIVE_ENCODER_PULSES_PER_REVOLUTION * Constants.DRIVE_SYSTEM_GEAR_RATIO));
        rightDriveEncoder.setPIDSourceType(PIDSourceType.kRate);
        rightDriveEncoder.setName("Drive Encoders", "Right");

        leftDriveController = new PIDController(Parameters.LEFT_DRIVE_P, Parameters.LEFT_DRIVE_I, Parameters.LEFT_DRIVE_D, leftDriveEncoder, leftDrive);
        leftDriveController.setName("Drive PID Controllers", "Left");

        rightDriveController = new PIDController(Parameters.RIGHT_DRIVE_P, Parameters.RIGHT_DRIVE_I, Parameters.RIGHT_DRIVE_D, rightDriveEncoder, rightDrive);
        rightDriveController.setName("Drive PID Controllers", "Right");

        liftSystem = new LiftSystem(table);

        climbMotor = new WPI_TalonSRX(Parameters.CLIMB_MOTOR_CAN_ID);
        climbMotor.setName("Climb Motor");

        imu = new PigeonIMU(Parameters.IMU_CAN_ID);
        PigeonGyro dashGyro = new PigeonGyro(imu);
        dashGyro.setName("Dashboard Gyro");
        ypr = new double[3];

        shifterSolenoid = new Solenoid(Parameters.PCM_CAN_ID, Parameters.SHIFTER_SOLENOID_CHANNEL);
        shifterSolenoid.setName("Pneumatics", "Shifter Solenoid");
        inLowGear = true;

        clawSolenoid = new Solenoid(Parameters.PCM_CAN_ID, Parameters.CLAW_SOLENOID_CHANNEL);
        clawSolenoid.setName("Pneumatics", "Claw Solenoid");

        coprocessor = new Rioduino();

        autoChooser = new SendableChooser<>();
        autoChooser.addObject("[A] Center - SWITCH", Autonomous.A_CENTER_SWITCH);
        autoChooser.addObject("[B] Left Side - SWITCH", Autonomous.B_LEFT_SIDE_SWITCH);
        autoChooser.addObject("[B] Right Side - SWITCH", Autonomous.B_RIGHT_SIDE_SWITCH);
        autoChooser.addObject("[C] Left Side - SCALE", Autonomous.C_LEFT_SIDE_SCALE);
        autoChooser.addObject("[C] Right Side - SCALE", Autonomous.C_RIGHT_SIDE_SCALE);
        autoChooser.addObject("[D] Cross Auto Line", Autonomous.D_CROSS_AUTO_LINE);
        autoChooser.addDefault("Disabled", Autonomous.DISABLED);
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
            table.getEntry("Auto Selected").setString("");
        }

        table.getSubTable("Status Flags").getEntry("In Low Gear").setBoolean(inLowGear);
        table.getSubTable("Status Flags").getEntry("Pac Man Mode").setBoolean(pacManMode);
        table.getSubTable("Status Flags").getEntry("Is Tipping").setBoolean(isTipping);

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

        leftDriveController.setEnabled(false);
        rightDriveController.setEnabled(false);

        liftSystem.resetEncoder();
        liftSystem.setPosition(LiftSystem.LiftPosition.BOTTOM);

        imu.setYaw(0, 0);

        shifterSolenoid.set(true); //low
        inLowGear = true;
        clawSolenoid.set(false); //closed

        switch (autoChooser.getSelected()) {
            case A_CENTER_SWITCH:
                state = AutoState.A1_CENTER_START;
                switch (MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR)) {
                    case LEFT: //Center, turn left to left plate
                        turnAngle = Parameters.AUTO_CENTER_LEFT_SWITCH_TURN_ANGLE;
                        break;
                    case RIGHT: //Center, turn right to right plate
                        turnAngle = Parameters.AUTO_CENTER_RIGHT_SWITCH_TURN_ANGLE;
                        break;
                    default: //We somehow don't know the angle...
                        turnAngle = Parameters.AUTO_CENTER_LEFT_SWITCH_TURN_ANGLE; //TODO set flag to never drop!
                        break;
                }
                break;
            case B_LEFT_SIDE_SWITCH:
                state = AutoState.B1_MOVE_TO_SWITCH;
                turnAngle = Parameters.AUTO_LEFT_SWITCH_TURN_ANGLE;
                break;
            case B_RIGHT_SIDE_SWITCH:
                state = AutoState.B1_MOVE_TO_SWITCH;
                turnAngle = Parameters.AUTO_RIGHT_SWITCH_TURN_ANGLE;
                break;
            case C_LEFT_SIDE_SCALE:
                if (MatchData.getOwnedSide(MatchData.GameFeature.SCALE) == MatchData.OwnedSide.LEFT) {
                    state = AutoState.C1_MOVE_TO_NULL_ZONE;
                    turnAngle = Parameters.AUTO_LEFT_SCALE_TURN_ANGLE;
                } else if (MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR) == MatchData.OwnedSide.LEFT) {
                    state = AutoState.B1_MOVE_TO_SWITCH; //Fall back to LEFT SIDE SWITCH
                    turnAngle = Parameters.AUTO_LEFT_SWITCH_TURN_ANGLE;
                } else { //Everything is bad
                    state = AutoState.D1_DRIVE_FORWARD;
                }
                break;
            case C_RIGHT_SIDE_SCALE:
                if (MatchData.getOwnedSide(MatchData.GameFeature.SCALE) == MatchData.OwnedSide.RIGHT) {
                    state = AutoState.C1_MOVE_TO_NULL_ZONE;
                    turnAngle = Parameters.AUTO_RIGHT_SCALE_TURN_ANGLE;
                } else if (MatchData.getOwnedSide(MatchData.GameFeature.SWITCH_NEAR) == MatchData.OwnedSide.RIGHT) {
                    state = AutoState.B1_MOVE_TO_SWITCH; //Fall back to RIGHT SIDE SWITCH
                    turnAngle = Parameters.AUTO_RIGHT_SWITCH_TURN_ANGLE;
                } else { //Everything is bad
                    state = AutoState.D1_DRIVE_FORWARD;
                }
                break;
            case D_CROSS_AUTO_LINE:
                state = AutoState.D1_DRIVE_FORWARD;
            default:
                state = AutoState.FINISHED;
                break;
        }
    }

    @Override
    public void autonomousPeriodic() {
        if (state != AutoState.FINISHED && !isTipping) {
            liftSystem.operateLift();

            // TIPPING PROTECTION - FAILSAFE AUTO
            // Disabled in specific (starting) states
            if (Math.abs(ypr[1]) >= 7 && state != AutoState.A1_CENTER_START
                    && state != AutoState.B1_MOVE_TO_SWITCH
                    && state != AutoState.C1_MOVE_TO_NULL_ZONE
                    && state != AutoState.D1_DRIVE_FORWARD) {

                clawSolenoid.set(true); //open

                isTipping = true;
                state = AutoState.C3_FAILSAFE_TIPPING_PROTECTION;
            }
        }

        switch (state) {

            // SWITCH
            case A1_CENTER_START:
                if (leftDriveEncoder.getDistance() > Parameters.AUTO_STARTING_DISTANCE
                        || rightDriveEncoder.getDistance() > Parameters.AUTO_STARTING_DISTANCE) {
                    robotDrive.stopMotor();
                    liftSystem.setPosition(LiftSystem.LiftPosition.AUTO);
                    state = AutoState.A1_TURN_TO_TARGET;
                } else {
                    robotDrive.arcadeDrive(Parameters.AUTO_SLOW_SPEED, 0, false);
                }
                break;
            case A1_TURN_TO_TARGET:
                double turnSpeed = (turnAngle - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(turnSpeed) > Parameters.AUTO_SLOW_SPEED) {
                    turnSpeed = Math.copySign(Parameters.AUTO_SLOW_SPEED, turnSpeed);
                }

                if (Math.abs(turnAngle - ypr[0]) <= Parameters.AUTO_ANGULAR_DEADBAND) {
                    robotDrive.stopMotor();

                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();

                    state = AutoState.A1_MOVE_TO_POSITION;
                } else {
                    robotDrive.arcadeDrive(0, -turnSpeed, false);
                }
                break;
            case A1_MOVE_TO_POSITION:
                if (leftDriveEncoder.getDistance() > Parameters.AUTO_POSITIONING_DISTANCE
                        || rightDriveEncoder.getDistance() > Parameters.AUTO_POSITIONING_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.A1_TURN_TO_ZERO;
                } else {
                    robotDrive.arcadeDrive(Parameters.AUTO_SLOW_SPEED, 0, false);
                }
                break;
            case A1_TURN_TO_ZERO:
                turnSpeed = (0 - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(turnSpeed) > Parameters.AUTO_SLOW_SPEED) {
                    turnSpeed = Math.copySign(Parameters.AUTO_SLOW_SPEED, turnSpeed);
                }

                if (Math.abs(0 - ypr[0]) <= Parameters.AUTO_ANGULAR_DEADBAND) {
                    robotDrive.stopMotor();
                    state = AutoState.A1_MOVE_TO_SWITCH;
                } else {
                    robotDrive.arcadeDrive(0, -turnSpeed, false);
                }
                break;
            case A1_MOVE_TO_SWITCH:
                int switchDistance = coprocessor.getSonarDistance();

                if (switchDistance > 0 && switchDistance < 8) {
                    robotDrive.stopMotor();
                    loopCounter = 0;
                    state = AutoState.A1_DEPOSIT_CUBE_ON_SWITCH;
                } else {
                    robotDrive.arcadeDrive(Parameters.AUTO_SLOW_SPEED, 0, false);
                }
                break;
            /*case A1_DALEK_MODE: // SEEK - LOCATE - DESTROY!
                int visionTargetOffset = coprocessor.getVisionTargetCoord() - 160;
                turnSpeed = ((double) visionTargetOffset) / 160;

                int followDistance = coprocessor.getSonarDistance();
                double followSpeed = ((double) followDistance) / 20;
                if (Math.abs(followSpeed) > Parameters.AUTO_SLOW_SPEED) {
                    followSpeed = Math.copySign(Parameters.AUTO_SLOW_SPEED, followSpeed);
                }

                if (followDistance > 0 && followDistance < 8) { //Reached target... EXTERMINATE!
                    robotDrive.stopMotor();
                    loopCounter = 0;
                    state = AutoState.A1_DEPOSIT_CUBE_ON_SWITCH;
                } else if (coprocessor.getVisionTargetCoord() < 400) { //todo consistent
                    robotDrive.arcadeDrive(followSpeed, turnSpeed, false);
                } else {
                    robotDrive.stopMotor();
                    state = AutoState.A3_FAILSAFE_TURN_TO_ZERO;
                }
                break;*/
            case A1_DEPOSIT_CUBE_ON_SWITCH:
                if (liftSystem.getEncoder().getRaw() > LiftSystem.LiftPosition.AUTO.getDistance() - (2 * Parameters.LIFT_SYSTEM_POSITION_DEADBAND)) {
                    robotDrive.arcadeDrive(0, 0);

                    clawSolenoid.set(true); //open

                    if (loopCounter >= Parameters.AUTO_CUBE_DROP_DELAY / 20) {
                        leftDriveEncoder.reset();
                        rightDriveEncoder.reset();

                        state = AutoState.A2_BACK_UP_FROM_TARGET;
                    } else {
                        loopCounter++;
                    }
                }
                break;

            // SWITCH CUBE 2: ELECTRIC BOOGALOO
            case A2_BACK_UP_FROM_TARGET:
                if (leftDriveEncoder.getDistance() < -Parameters.AUTO_STARTING_DISTANCE
                        || rightDriveEncoder.getDistance() < -Parameters.AUTO_STARTING_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.A2_TURN_AWAY_FROM_TARGET;
                } else {
                    robotDrive.arcadeDrive(-Parameters.AUTO_SLOW_SPEED, 0, false);
                }
                break;
            case A2_TURN_AWAY_FROM_TARGET:
                turnSpeed = (Math.copySign(90, turnAngle) - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(turnSpeed) > Parameters.AUTO_SLOW_SPEED) {
                    turnSpeed = Math.copySign(Parameters.AUTO_SLOW_SPEED, turnSpeed);
                }

                if (Math.abs(Math.copySign(90, turnAngle) - ypr[0]) > Parameters.AUTO_ANGULAR_DEADBAND) { //todo consistent
                    robotDrive.arcadeDrive(0, -turnSpeed, false);
                } else {
                    robotDrive.stopMotor();
                    liftSystem.setPosition(LiftSystem.LiftPosition.BOTTOM);

                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();

                    state = AutoState.A2_MOVE_PAST_TARGET;
                }
                break;
            case A2_MOVE_PAST_TARGET:
                if (leftDriveEncoder.getDistance() > Parameters.AUTO_PAST_TARGET_DISTANCE
                        || rightDriveEncoder.getDistance() > Parameters.AUTO_PAST_TARGET_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.A2_TURN_TO_ALLIANCE;
                } else {
                    robotDrive.arcadeDrive(Parameters.AUTO_SLOW_SPEED, 0, false);
                }
                break;
            case A2_TURN_TO_ALLIANCE:
                turnSpeed = (Math.copySign(180, turnAngle) - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(turnSpeed) > Parameters.AUTO_SLOW_SPEED) {
                    turnSpeed = Math.copySign(Parameters.AUTO_SLOW_SPEED, turnSpeed);
                }

                if (Math.abs(Math.copySign(180, turnAngle) - ypr[0]) <= Parameters.AUTO_ANGULAR_DEADBAND) {
                    robotDrive.stopMotor();

                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();

                    state = AutoState.A2_MOVE_PAST_SWITCH;
                } else {
                    robotDrive.arcadeDrive(0, -turnSpeed, false);
                }
                break;
            case A2_MOVE_PAST_SWITCH:
                if (leftDriveEncoder.getDistance() < -Parameters.AUTO_PAST_SWITCH_DISTANCE
                        || rightDriveEncoder.getDistance() < -Parameters.AUTO_PAST_SWITCH_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.PAC_MAN_MODE;
                } else {
                    robotDrive.arcadeDrive(-Parameters.AUTO_SLOW_SPEED, 0, false);
                }
                break;

            // SWITCH FAILSAFE: GET PAST THAT LINE
            case A3_FAILSAFE_TURN_TO_ZERO:
                turnSpeed = (0 - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(turnSpeed) > Parameters.AUTO_SLOW_SPEED) {
                    turnSpeed = Math.copySign(Parameters.AUTO_SLOW_SPEED, turnSpeed);
                }

                if (Math.abs(0 - ypr[0]) <= Parameters.AUTO_ANGULAR_DEADBAND) {
                    robotDrive.stopMotor();

                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();

                    state = AutoState.A3_FAILSAFE_DRIVE_FORWARD;
                } else {
                    robotDrive.arcadeDrive(0, -turnSpeed, false);
                }
                break;
            case A3_FAILSAFE_DRIVE_FORWARD:
                if (leftDriveEncoder.getDistance() > Parameters.AUTO_FAILSAFE_DISTANCE
                        || rightDriveEncoder.getDistance() > Parameters.AUTO_FAILSAFE_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.FINISHED;
                } else {
                    robotDrive.arcadeDrive(Parameters.AUTO_SLOW_SPEED, 0, false);
                }

                // SIDE SWITCH
            case B1_MOVE_TO_SWITCH:
                if (leftDriveEncoder.getDistance() > Parameters.AUTO_SWITCH_DISTANCE
                        || rightDriveEncoder.getDistance() > Parameters.AUTO_SWITCH_DISTANCE) {
                    robotDrive.stopMotor();
                    liftSystem.setPosition(LiftSystem.LiftPosition.AUTO);

                    state = AutoState.B1_TURN_TO_SWITCH;
                } else {
                    double approachSpeed;
                    if (leftDriveEncoder.getDistance() > Parameters.AUTO_SWITCH_DISTANCE * 0.75
                            || rightDriveEncoder.getDistance() > Parameters.AUTO_SWITCH_DISTANCE * 0.75) { //Only slow down in last 25% of run
                        approachSpeed = Parameters.AUTO_MAX_SPEED - ((leftDriveEncoder.getDistance() - (Parameters.AUTO_SWITCH_DISTANCE * 0.75))
                                / (Parameters.AUTO_SWITCH_DISTANCE * 0.25));

                        if (approachSpeed < Parameters.AUTO_SLOW_SPEED) {
                            approachSpeed = Parameters.AUTO_SLOW_SPEED;
                        }
                    } else {
                        approachSpeed = Parameters.AUTO_MAX_SPEED;
                    }

                    double gyroTurnCorrection = ypr[0] / 30; //Keep the robot driving straight!
                    robotDrive.arcadeDrive(approachSpeed, gyroTurnCorrection, false);
                }
                break;
            case B1_TURN_TO_SWITCH:
                turnSpeed = (turnAngle - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(turnSpeed) > Parameters.AUTO_SLOW_SPEED) {
                    turnSpeed = Math.copySign(Parameters.AUTO_SLOW_SPEED, turnSpeed);
                }

                if (Math.abs(turnAngle - ypr[0]) > Parameters.AUTO_ANGULAR_DEADBAND) { //todo consistent
                    robotDrive.arcadeDrive(0, -turnSpeed, false);
                } else {
                    robotDrive.arcadeDrive(0, 0);

                    // Don't move forward until the lift is up all the way
                    if (liftSystem.getEncoder().getRaw() >
                            LiftSystem.LiftPosition.AUTO.getDistance() - (2 * Parameters.LIFT_SYSTEM_POSITION_DEADBAND)) {

                        leftDriveEncoder.reset();
                        rightDriveEncoder.reset();

                        state = AutoState.B1_APPROACH_SWITCH;
                    }
                }
                break;
            case B1_APPROACH_SWITCH:
                switchDistance = coprocessor.getSonarDistance();

                if (switchDistance > 0 && switchDistance < 8) {
                    robotDrive.stopMotor();
                    loopCounter = 0;
                    state = AutoState.B1_DEPOSIT_CUBE_ON_SWITCH;
                } else {
                    robotDrive.arcadeDrive(Parameters.AUTO_SLOW_SPEED, 0, false);
                }
                break;
            case B1_DEPOSIT_CUBE_ON_SWITCH:
                robotDrive.arcadeDrive(0, 0);

                clawSolenoid.set(true); //open

                if (loopCounter >= Parameters.AUTO_CUBE_DROP_DELAY / 20) {
                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();

                    state = AutoState.B2_BACK_UP_FROM_SWITCH;
                } else {
                    loopCounter++;
                }
                break;

            // SWITCH CUBE 2, NOW ON SIDE
            case B2_BACK_UP_FROM_SWITCH:
                if (leftDriveEncoder.getDistance() < -Parameters.AUTO_STARTING_DISTANCE
                        || rightDriveEncoder.getDistance() < -Parameters.AUTO_STARTING_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.B2_TURN_TO_ALLIANCE;
                } else {
                    robotDrive.arcadeDrive(-Parameters.AUTO_SLOW_SPEED, 0, false);
                }
                break;
            case B2_TURN_TO_ALLIANCE:
                turnSpeed = (Math.copySign(180, turnAngle) - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(turnSpeed) > Parameters.AUTO_SLOW_SPEED) {
                    turnSpeed = Math.copySign(Parameters.AUTO_SLOW_SPEED, turnSpeed);
                }

                if (Math.abs(Math.copySign(180, turnAngle) - ypr[0]) <= Parameters.AUTO_ANGULAR_DEADBAND) {
                    robotDrive.stopMotor();
                    liftSystem.setPosition(LiftSystem.LiftPosition.BOTTOM);

                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();

                    state = AutoState.B2_MOVE_PAST_SWITCH;
                } else {
                    robotDrive.arcadeDrive(0, -turnSpeed, false);
                }
                break;
            case B2_MOVE_PAST_SWITCH:
                if (leftDriveEncoder.getDistance() < -Parameters.AUTO_PAST_SWITCH_SHORT_DISTANCE
                        || rightDriveEncoder.getDistance() < -Parameters.AUTO_PAST_SWITCH_SHORT_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.PAC_MAN_MODE;
                } else {
                    robotDrive.arcadeDrive(-Parameters.AUTO_SLOW_SPEED, 0, false);
                }
                break;

            // SCALE
            case C1_MOVE_TO_NULL_ZONE:
                if (leftDriveEncoder.getDistance() > Parameters.AUTO_NULL_ZONE_DISTANCE
                        || rightDriveEncoder.getDistance() > Parameters.AUTO_NULL_ZONE_DISTANCE) {
                    robotDrive.stopMotor();
                    liftSystem.setPosition(LiftSystem.LiftPosition.SCALE);

                    state = AutoState.C1_TURN_TO_SCALE;
                } else {
                    double approachSpeed;
                    if (leftDriveEncoder.getDistance() > Parameters.AUTO_NULL_ZONE_DISTANCE * 0.75
                            || rightDriveEncoder.getDistance() > Parameters.AUTO_NULL_ZONE_DISTANCE * 0.75) { //Only slow down in last 25% of run
                        approachSpeed = Parameters.AUTO_MAX_SPEED - ((leftDriveEncoder.getDistance() - (Parameters.AUTO_NULL_ZONE_DISTANCE * 0.75))
                                / (Parameters.AUTO_NULL_ZONE_DISTANCE * 0.25));

                        if (approachSpeed < Parameters.AUTO_SLOW_SPEED) {
                            approachSpeed = Parameters.AUTO_SLOW_SPEED;
                        }
                    } else {
                        approachSpeed = Parameters.AUTO_MAX_SPEED;
                    }

                    double gyroTurnCorrection = ypr[0] / 30; //Keep the robot driving straight!
                    robotDrive.arcadeDrive(approachSpeed, gyroTurnCorrection, false);
                }
                break;
            case C1_TURN_TO_SCALE:
                turnSpeed = (turnAngle - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(turnSpeed) > Parameters.AUTO_SLOW_SPEED) {
                    turnSpeed = Math.copySign(Parameters.AUTO_SLOW_SPEED, turnSpeed);
                }

                if (Math.abs(turnAngle - ypr[0]) > Parameters.AUTO_ANGULAR_DEADBAND) { //todo consistent
                    robotDrive.arcadeDrive(0, -turnSpeed, false);
                } else {
                    robotDrive.arcadeDrive(0, 0);

                    // Don't move forward until the lift is up all the way
                    if (liftSystem.getEncoder().getRaw() > LiftSystem.LiftPosition.SCALE.getDistance() - (2 * Parameters.LIFT_SYSTEM_POSITION_DEADBAND)) {

                        leftDriveEncoder.reset();
                        rightDriveEncoder.reset();

                        state = AutoState.C1_APPROACH_SCALE;
                    }
                }
                break;
            case C1_APPROACH_SCALE:
                if (leftDriveEncoder.getDistance() > Parameters.AUTO_APPROACH_DISTANCE
                        || rightDriveEncoder.getDistance() > Parameters.AUTO_APPROACH_DISTANCE) {
                    robotDrive.stopMotor();
                    loopCounter = 0;
                    state = AutoState.C1_DEPOSIT_CUBE_ON_SCALE;
                } else {
                    robotDrive.arcadeDrive(Parameters.AUTO_SLOW_SPEED, 0, false);
                }
                break;
            case C1_DEPOSIT_CUBE_ON_SCALE:
                robotDrive.arcadeDrive(0, 0);

                clawSolenoid.set(true); //open

                if (loopCounter >= Parameters.AUTO_CUBE_DROP_DELAY / 20) {
                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();

                    state = AutoState.C1_BACK_UP_FROM_SCALE;
                } else {
                    loopCounter++;
                }
                break;
            case C1_BACK_UP_FROM_SCALE:
                if (leftDriveEncoder.getDistance() < -Parameters.AUTO_APPROACH_DISTANCE
                        || rightDriveEncoder.getDistance() < -Parameters.AUTO_APPROACH_DISTANCE) {
                    robotDrive.stopMotor();
                    liftSystem.setPosition(LiftSystem.LiftPosition.BOTTOM);
                    state = AutoState.C2_TURN_TO_SWITCH;
                } else {
                    robotDrive.arcadeDrive(-Parameters.AUTO_SUPER_SLOW_SPEED, 0, false);
                }
                break;

            // SCALE CUBE TWO: SOMEHOW WE GOT THIS FAR
            case C2_TURN_TO_SWITCH:
                turnSpeed = (Math.copySign(180, turnAngle) - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(turnSpeed) > Parameters.AUTO_SLOW_SPEED) {
                    turnSpeed = Math.copySign(Parameters.AUTO_SLOW_SPEED, turnSpeed);
                }

                if (Math.abs(Math.copySign(180, turnAngle) - ypr[0]) <= Parameters.AUTO_ANGULAR_DEADBAND
                        && liftSystem.getEncoder().getRaw() < LiftSystem.LiftPosition.BOTTOM.getDistance() + Parameters.LIFT_SYSTEM_POSITION_DEADBAND) {

                    robotDrive.stopMotor();

                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();

                    state = AutoState.PAC_MAN_MODE;
                } else {
                    robotDrive.arcadeDrive(0, -turnSpeed, false);
                }
                break;

            // SCALE FAILSAFE: TIPPING PROTECTION
            case C3_FAILSAFE_TIPPING_PROTECTION:
                loopCounter = 0;
                state = AutoState.C3_FAILSAFE_DELAY;
                break;
            case C3_FAILSAFE_DELAY:
                if (loopCounter >= Parameters.AUTO_TIP_CORRECTION_DELAY / 20) {
                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();

                    state = AutoState.C3_FAILSAFE_TIP_CORRECTION;
                } else {
                    loopCounter++;
                }
            case C3_FAILSAFE_TIP_CORRECTION:
                if (Math.abs(ypr[1]) < 2) {
                    leftDriveEncoder.reset();
                    rightDriveEncoder.reset();

                    state = AutoState.C3_FAILSAFE_BACK_AWAY;
                } else {
                    robotDrive.arcadeDrive(Math.copySign(0.2, ypr[1]), 0, false);
                }
                break;
            case C3_FAILSAFE_BACK_AWAY:
                if (leftDriveEncoder.getDistance() < -2.0
                        || rightDriveEncoder.getDistance() < -2.0) {
                    robotDrive.stopMotor();
                    isTipping = false;
                    state = AutoState.FINISHED;
                } else {
                    robotDrive.arcadeDrive(-0.2, 0, false);
                }
                break;

            // CROSS AUTO LINE BECAUSE WE'RE DESPERATE
            case D1_DRIVE_FORWARD:
                if (leftDriveEncoder.getDistance() > Parameters.AUTO_DRIVE_FORWARD_DISTANCE
                        || rightDriveEncoder.getDistance() > Parameters.AUTO_DRIVE_FORWARD_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.FINISHED;
                } else {
                    double gyroTurnCorrection = ypr[0] / 30; //Keep the robot driving straight!
                    robotDrive.arcadeDrive(Parameters.AUTO_MAX_SPEED, gyroTurnCorrection, false);
                }
                break;

            // PAC MAN MODE FOR CUBE ACQUISITION
            case PAC_MAN_MODE:
                int powerCubeOffset = coprocessor.getPowerCubeCoord() - 160;
                turnSpeed = ((double) powerCubeOffset) / 160;

                int followDistance = coprocessor.getSonarDistance();
                double followSpeed = ((double) followDistance) / 20;
                if (Math.abs(followSpeed) > 0.6) {
                    followSpeed = Math.copySign(0.6, followSpeed);
                }

                if (coprocessor.getSonarDistance() > 0 && coprocessor.getSonarDistance() < 8) { //Cube in mouth... eat it! - TODO consistent
                    robotDrive.stopMotor();
                    state = AutoState.EAT_CUBE;
                } else if (coprocessor.getPowerCubeCoord() >= 0 && coprocessor.getPowerCubeCoord() < 400) {
                    robotDrive.arcadeDrive(followSpeed, turnSpeed, false);
                } else {
                    robotDrive.stopMotor();
                    state = AutoState.FINISHED;
                }
                break;
            case EAT_CUBE:
                robotDrive.arcadeDrive(0, 0);
                clawSolenoid.set(false); //eat the cube
                state = AutoState.FINISHED;
                break;

            // FINISHED (REDUNDANT COMMENT)
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

        leftDriveController.setEnabled(Parameters.DRIVE_PID_ENABLED);
        leftDriveController.setInputRange(-Parameters.PID_MAX_SPEED_LOW_GEAR, Parameters.PID_MAX_SPEED_LOW_GEAR);

        rightDriveController.setEnabled(Parameters.DRIVE_PID_ENABLED);
        rightDriveController.setInputRange(-Parameters.PID_MAX_SPEED_LOW_GEAR, Parameters.PID_MAX_SPEED_LOW_GEAR);

        imu.setYaw(0, 0);

        shifterSolenoid.set(true); //low
        clawSolenoid.set(false); //closed
    }

    @Override
    public void teleopPeriodic() {
        if (isTipping) { // TIPPING PROTECTION - OVERRIDE driver input
            robotDrive.arcadeDrive(Math.copySign(0.4, ypr[1]), 0, false);
        } else {
            if (Parameters.DRIVE_PID_ENABLED) { //TODO DO NOT USE THIS IS NOT COMPLETE
                double maxSpeed = inLowGear ? Parameters.PID_MAX_SPEED_LOW_GEAR : Parameters.PID_MAX_SPEED_HIGH_GEAR;

                leftDriveController.setSetpoint(-driveStick.getY() * maxSpeed); //TODO implement steering wheel
                rightDriveController.setSetpoint(-driveStick.getY() * maxSpeed);
            } else {
                robotDrive.arcadeDrive(-driveStick.getY(), driveWheel.getX());
            }
        }

        if (driveWheel.getRawButton(5)) { //Driver override for tipping protection - left wheel paddle
            isTipping = false;
        } else {
            if (Math.abs(ypr[1]) >= 10) { //Activate tipping protection
                isTipping = true;
                pacManMode = false;

                clawSolenoid.set(true); //open
                liftSystem.setPosition(LiftSystem.LiftPosition.BOTTOM);
            } else if (Math.abs(ypr[1]) < 1 && isTipping) {
                isTipping = false;
            }
        }

        liftSystem.operateLift(operatorController);

        // CLIMBER CONTROL
        if (prajBox.getRawButton(1) && Math.abs(operatorController.getRawAxis(5)) > 0.2) {
            if (prajBox.getRawButton(4)) { //Only allow backdrive if an extra button is being held
                climbMotor.set(operatorController.getRawAxis(5));
            } else {
                climbMotor.set(-Math.abs(operatorController.getRawAxis(5)));
            }
        } else {
            climbMotor.set(0);
        }

        if (operatorController.getRawButton(2)) {
            System.out.println("Entering PAC MAN mode");
            System.out.println("wakawakawakawakawaka");
            liftSystem.setPosition(LiftSystem.LiftPosition.BOTTOM);
            clawSolenoid.set(true); //open
            pacManMode = true;
        }

        if (operatorController.getRawButton(4)) {
            System.out.println("Leaving PAC MAN mode");
            pacManMode = false;
        }

        if (operatorController.getRawButtonPressed(7)) {
            shifterSolenoid.set(true); //low
            inLowGear = true;
        }

        if (operatorController.getRawButtonPressed(8)) {
            shifterSolenoid.set(false); //high
            inLowGear = false;
        }

        if (operatorController.getRawButton(6)) {
            liftSystem.setPosition(LiftSystem.LiftPosition.BOTTOM);
        }

        if (operatorController.getRawButton(1)) {
            liftSystem.setPosition(LiftSystem.LiftPosition.SWITCH);
        }

        if (operatorController.getRawButton(5)) {
            liftSystem.setPosition(LiftSystem.LiftPosition.HOLD_CURRENT); //huehuehue
        }

        if (operatorController.getRawAxis(2) > 0.9) {
            clawSolenoid.set(true); //open
        }

        if (operatorController.getRawAxis(3) > 0.9) {
            clawSolenoid.set(false); //closed
        }

        if (operatorController.getRawButtonPressed(9)) { //TODO disable
            imu.setYaw(0, 0);
        }

        if (operatorController.getRawButtonPressed(10)) { //TODO disable
            liftSystem.resetEncoder();
        }

        // AUTOMATIC SHIFTING
        if (liftSystem.getEncoder().getRaw() > Parameters.LIFT_ENCODER_NO_SHIFT_THRESHOLD && !inLowGear) {
            inLowGear = true;
            shifterSolenoid.set(true);
            leftDriveController.setInputRange(-Parameters.PID_MAX_SPEED_LOW_GEAR, Parameters.PID_MAX_SPEED_LOW_GEAR);
            rightDriveController.setInputRange(-Parameters.PID_MAX_SPEED_LOW_GEAR, Parameters.PID_MAX_SPEED_LOW_GEAR);
        } else {
            if (Math.abs(leftDriveEncoder.getRate()) > Parameters.UPPER_SHIFT_THRESHOLD
                    && Math.abs(rightDriveEncoder.getRate()) > Parameters.UPPER_SHIFT_THRESHOLD && inLowGear) {
                inLowGear = false;
                shifterSolenoid.set(false);
                leftDriveController.setInputRange(-Parameters.PID_MAX_SPEED_HIGH_GEAR, Parameters.PID_MAX_SPEED_HIGH_GEAR);
                rightDriveController.setInputRange(-Parameters.PID_MAX_SPEED_HIGH_GEAR, Parameters.PID_MAX_SPEED_HIGH_GEAR);
            } else if (Math.abs(leftDriveEncoder.getRate()) < Parameters.LOWER_SHIFT_THRESHOLD
                    && Math.abs(rightDriveEncoder.getRate()) < Parameters.LOWER_SHIFT_THRESHOLD && !inLowGear) {
                inLowGear = true;
                shifterSolenoid.set(true);
                leftDriveController.setInputRange(-Parameters.PID_MAX_SPEED_LOW_GEAR, Parameters.PID_MAX_SPEED_LOW_GEAR);
                rightDriveController.setInputRange(-Parameters.PID_MAX_SPEED_LOW_GEAR, Parameters.PID_MAX_SPEED_LOW_GEAR);
            }
        }

        //New Power Cube Eating Mode - PAC MAN!
        if (pacManMode) {
            int visionTargetOffset = coprocessor.getPowerCubeCoord() - 160;
            double turnSpeed = ((double) visionTargetOffset) / 160;

            int followDistance = coprocessor.getSonarDistance();
            double followSpeed = ((double) followDistance) / 20;
            if (Math.abs(followSpeed) > 0.6) {
                followSpeed = Math.copySign(0.6, followSpeed);
            }

            System.out.println(coprocessor.getSonarDistance());
            if (coprocessor.getSonarDistance() > 0 && coprocessor.getSonarDistance() < 8) { //Cube in mouth... eat it!
                System.out.println("cube found - nomnomnom");
                robotDrive.stopMotor();
                clawSolenoid.set(false); //eat the cube
                pacManMode = false;
            } else if (coprocessor.getPowerCubeCoord() >= 0 && coprocessor.getPowerCubeCoord() < 400) {
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
        isTipping = false;
    }

    public enum Autonomous {
        A_CENTER_SWITCH,

        B_LEFT_SIDE_SWITCH,
        B_RIGHT_SIDE_SWITCH,

        C_LEFT_SIDE_SCALE,
        C_RIGHT_SIDE_SCALE,

        D_CROSS_AUTO_LINE,

        DISABLED
    }

    public enum AutoState {
        A1_CENTER_START,
        A1_TURN_TO_TARGET,
        A1_MOVE_TO_POSITION,
        A1_TURN_TO_ZERO,
        A1_MOVE_TO_SWITCH,
        //A1_DALEK_MODE,
        A1_DEPOSIT_CUBE_ON_SWITCH,

        A2_BACK_UP_FROM_TARGET,
        A2_TURN_AWAY_FROM_TARGET,
        A2_MOVE_PAST_TARGET,
        A2_TURN_TO_ALLIANCE,
        A2_MOVE_PAST_SWITCH,

        A3_FAILSAFE_TURN_TO_ZERO,
        A3_FAILSAFE_DRIVE_FORWARD,

        B1_MOVE_TO_SWITCH,
        B1_TURN_TO_SWITCH,
        B1_APPROACH_SWITCH,
        B1_DEPOSIT_CUBE_ON_SWITCH,

        B2_BACK_UP_FROM_SWITCH,
        B2_TURN_TO_ALLIANCE,
        B2_MOVE_PAST_SWITCH,

        C1_MOVE_TO_NULL_ZONE,
        C1_TURN_TO_SCALE,
        C1_APPROACH_SCALE,
        C1_DEPOSIT_CUBE_ON_SCALE,
        C1_BACK_UP_FROM_SCALE,

        C2_TURN_TO_SWITCH,

        C3_FAILSAFE_TIPPING_PROTECTION,
        C3_FAILSAFE_DELAY,
        C3_FAILSAFE_TIP_CORRECTION,
        C3_FAILSAFE_BACK_AWAY,

        D1_DRIVE_FORWARD,

        PAC_MAN_MODE,
        EAT_CUBE,

        FINISHED
    }
}
