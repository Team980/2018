package com.team980.robot2018;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team980.robot2018.sensors.LaserRangefinder;
import com.team980.robot2018.sensors.Rioduino;
import com.team980.robot2018.util.PigeonGyro;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import openrio.powerup.MatchData;

public class Robot extends IterativeRobot { //TODO test TimedRobot - exact 20ms vs about 20ms

    private Joystick driveStick;
    private Joystick driveWheel;
    private Joystick operatorController;

    private SpeedControllerGroup leftDrive;
    private SpeedControllerGroup rightDrive;
    private DifferentialDrive robotDrive;

    //private Encoder leftDriveEncoder;
    //private Encoder rightDriveEncoder;

    private WPI_TalonSRX liftMotor;
    private int upwardAccelerationCounter;

    private AnalogInput lowerProximitySensor;
    private AnalogInput upperProximitySensor;
    private LaserRangefinder rangefinder;

    private PigeonIMU imu;
    private double[] ypr;

    private DoubleSolenoid shifterSolenoid;

    private DoubleSolenoid clawSolenoid;

    private Rioduino coprocessor;

    private Relay dalekEye;

    private NetworkTable table;

    private SendableChooser<Autonomous> autoChooser;
    private int turnAngle;
    private AutoState state;

    private boolean inLowGear = true;
    private LiftState liftState = LiftState.STOPPED;
    private boolean pacManMode = false;

    private PowerDistributionPanel pdp;

    @Override
    public void robotInit() {
        driveStick = new Joystick(Parameters.DRIVE_STICK_JS_ID);
        driveWheel = new Joystick(Parameters.DRIVE_WHEEL_JS_ID);
        operatorController = new Joystick(Parameters.OPERATOR_CONTROLLER_JS_ID);

        leftDrive = new SpeedControllerGroup(new WPI_TalonSRX(Parameters.LEFT_FRONT_DRIVE_CAN_ID), new WPI_TalonSRX(Parameters.LEFT_BACK_DRIVE_CAN_ID));
        leftDrive.setInverted(true);
        rightDrive = new SpeedControllerGroup(new WPI_TalonSRX(Parameters.RIGHT_FRONT_DRIVE_CAN_ID), new WPI_TalonSRX(Parameters.RIGHT_BACK_DRIVE_CAN_ID));
        rightDrive.setInverted(true);
        robotDrive = new DifferentialDrive(leftDrive, rightDrive);
        robotDrive.setName("Robot Drive");

        /*leftDriveEncoder = new Encoder(Parameters.LEFT_ENCODER_DIO_CHANNEL_A, Parameters.LEFT_ENCODER_DIO_CHANNEL_B, Parameters.INVERT_LEFT_ENCODER, CounterBase.EncodingType.k4X);
        leftDriveEncoder.setDistancePerPulse((2 * (Constants.PI) * (Constants.wheelRadius / 12)) / (Constants.encoderPulsesPerRevolution));
        leftDriveEncoder.setPIDSourceType(PIDSourceType.kRate);
        leftDriveEncoder.setName("Encoders", "Left");

        rightDriveEncoder = new Encoder(Parameters.RIGHT_ENCODER_DIO_CHANNEL_A, Parameters.RIGHT_ENCODER_DIO_CHANNEL_B, Parameters.INVERT_RIGHT_ENCODER, CounterBase.EncodingType.k4X);
        rightDriveEncoder.setDistancePerPulse((2 * (Constants.PI) * (Constants.wheelRadius / 12)) / (Constants.encoderPulsesPerRevolution));
        rightDriveEncoder.setPIDSourceType(PIDSourceType.kRate);
        rightDriveEncoder.setName("Encoders", "Right");*/

        liftMotor = new WPI_TalonSRX(Parameters.LIFT_MOTOR_CAN_ID);
        liftMotor.setName("Lift Motor");

        lowerProximitySensor = new AnalogInput(Parameters.LOWER_PROXIMITY_SENSOR_ANALOG_CHANNEL);
        upperProximitySensor = new AnalogInput(Parameters.UPPER_PROXIMITY_SENSOR_ANALOG_CHANNEL);
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

        table = NetworkTableInstance.getDefault().getTable("ThunderBots");

        autoChooser = new SendableChooser<>();
        autoChooser.addDefault("Disabled", Autonomous.DISABLED);
        autoChooser.addObject("Left Side - Cube Drop", Autonomous.LEFT_SIDE_CUBE_DROP);
        autoChooser.addObject("Right Side - Cube Drop", Autonomous.RIGHT_SIDE_CUBE_DROP);
        autoChooser.addObject("Center - Cube Drop", Autonomous.CENTER_CUBE_DROP);
        autoChooser.addObject("Far Left - Get To Scale", Autonomous.FAR_LEFT_GET_TO_SCALE);
        autoChooser.setName("Autonomous Chooser");
        LiveWindow.add(autoChooser); //This actually works

        table.getEntry("Autonomous State").setString("");

        pdp = new PowerDistributionPanel(); //TODO fix Shuffleboard readings
        pdp.clearStickyFaults();
        pdp.resetTotalEnergy();
    }

    @Override
    public void robotPeriodic() {
        imu.getYawPitchRoll(ypr);
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

        table.getSubTable("Lift System").getEntry("Lower Proximity Sensor").setBoolean(lowerProximitySensor.getVoltage() < Parameters.PROXIMITY_SENSOR_THRESHOLD);
        table.getSubTable("Lift System").getEntry("Upper Proximity Sensor").setBoolean(upperProximitySensor.getVoltage() < Parameters.PROXIMITY_SENSOR_THRESHOLD);
        table.getSubTable("Lift System").getEntry("Laser Rangefinder").setNumber(rangefinder.getDistance());
        table.getSubTable("Lift System").getEntry("Lift Motor Current").setNumber(pdp.getCurrent(Parameters.LIFT_MOTOR_PDP_CHANNEL));

        table.getSubTable("Coprocessor").getEntry("Vision Target Coord").setNumber(coprocessor.getVisionTargetCoord());
        table.getSubTable("Coprocessor").getEntry("Power Cube Width").setNumber(coprocessor.getPowerCubeWidth());
        table.getSubTable("Coprocessor").getEntry("Power Cube Height").setNumber(coprocessor.getPowerCubeHeight());
        table.getSubTable("Coprocessor").getEntry("Power Cube Coord").setNumber(coprocessor.getPowerCubeCoord());
        table.getSubTable("Coprocessor").getEntry("Ranged Distance").setNumber(coprocessor.getRangedDistance());
    }

    @Override
    public void autonomousInit() {
        //leftDriveEncoder.reset();
        //rightDriveEncoder.reset();

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
    public void autonomousPeriodic() {
        switch (state) {
            case START:
                if (coprocessor.getRangedDistance() > Parameters.AUTO_STARTING_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.TURN_TO_ANGLE_AND_LIFT;
                } else {
                    liftState = LiftState.UP;
                    robotDrive.arcadeDrive(-Parameters.AUTO_MAX_SPEED, 0, false);
                }
                break;
            case TURN_TO_ANGLE_AND_LIFT:
                if (liftState == LiftState.UP && upperProximitySensor.getVoltage() > Parameters.PROXIMITY_SENSOR_THRESHOLD
                        && pdp.getCurrent(Parameters.LIFT_MOTOR_PDP_CHANNEL) < Parameters.LIFT_MOTOR_CURRENT_THRESHOLD) {
                    upwardAccelerationCounter++;
                    double speed = Parameters.LIFT_MOTOR_MIN_UPWARD_SPEED + (Parameters.LIFT_MOTOR_UPWARD_ACCELERATION * upwardAccelerationCounter);
                    if (speed < Parameters.LIFT_MOTOR_MAX_UPWARD_SPEED) {
                        liftMotor.set(speed);
                    } else {
                        liftMotor.set(Parameters.LIFT_MOTOR_MAX_UPWARD_SPEED);
                    }
                }

                double turnSpeed = (turnAngle - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(turnAngle - ypr[0]) > Parameters.AUTO_ANGULAR_DEADBAND) { //todo consistent
                    robotDrive.arcadeDrive(0, turnSpeed, false);
                } else if (upperProximitySensor.getVoltage() < Parameters.PROXIMITY_SENSOR_THRESHOLD) {
                    robotDrive.stopMotor();

                    liftMotor.set(0);
                    liftState = LiftState.STOPPED;

                    state = AutoState.MOVE_TO_POSITION;
                }
                break;
            case MOVE_TO_POSITION:
                if (coprocessor.getRangedDistance() > Parameters.AUTO_POSITIONING_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.DALEK_MODE;
                } else {
                    robotDrive.arcadeDrive(-Parameters.AUTO_MAX_SPEED, 0, false);
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
            case DEPOSIT_CUBE:
                if (upperProximitySensor.getVoltage() < Parameters.PROXIMITY_SENSOR_THRESHOLD) {
                    clawSolenoid.set(DoubleSolenoid.Value.kForward); //open
                    state = AutoState.FINISHED;
                }
                break;
            case BACKUP_TURN_TO_ZERO:
                turnSpeed = (0 - ypr[0]) / Parameters.AUTO_ANGULAR_SPEED_FACTOR;

                if (Math.abs(0 - ypr[0]) > Parameters.AUTO_ANGULAR_DEADBAND) { //todo consistent
                    robotDrive.arcadeDrive(0, turnSpeed, false);
                } else {
                    robotDrive.stopMotor();
                    state = AutoState.BACKUP_DRIVE_FORWARD;
                }
                break;
            case BACKUP_DRIVE_FORWARD:
                if (coprocessor.getRangedDistance() > Parameters.AUTO_BACKUP_DISTANCE) {
                    robotDrive.stopMotor();
                    state = AutoState.FINISHED;
                } else {
                    robotDrive.arcadeDrive(-Parameters.AUTO_MAX_SPEED, 0, false);
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
        //leftDriveEncoder.reset();
        //rightDriveEncoder.reset();

        imu.setYaw(0, 0);

        shifterSolenoid.set(DoubleSolenoid.Value.kForward); //low
        clawSolenoid.set(DoubleSolenoid.Value.kReverse); //closed

        dalekEye.set(Relay.Value.kOff);
    }

    @Override
    public void teleopPeriodic() {

        switch (Parameters.CONTROL_MODE) {
            case COMPETITION_DRIVER_STATION:
                teleopOperatorControls(operatorController);
                robotDrive.arcadeDrive(-driveStick.getY(), driveWheel.getX());
                break;
            case SINGLE_JOYSTICK:
                teleopOperatorControls(driveStick);
                robotDrive.arcadeDrive(-driveStick.getY(), driveStick.getX()); //Z axis is improperly calibrated >:(
                break;
            case GAME_CONTROLLER:
                teleopOperatorControls(operatorController);
                robotDrive.arcadeDrive(-(operatorController.getRawAxis(3) - operatorController.getRawAxis(2)), operatorController.getRawAxis(0));
                break;
        }

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

        // LIFT SYSTEM
        if (liftState == LiftState.UP && upperProximitySensor.getVoltage() > Parameters.PROXIMITY_SENSOR_THRESHOLD
                && pdp.getCurrent(Parameters.LIFT_MOTOR_PDP_CHANNEL) < Parameters.LIFT_MOTOR_CURRENT_THRESHOLD) {
            upwardAccelerationCounter++;
            double speed = Parameters.LIFT_MOTOR_MIN_UPWARD_SPEED + (Parameters.LIFT_MOTOR_UPWARD_ACCELERATION * upwardAccelerationCounter);
            if (speed < Parameters.LIFT_MOTOR_MAX_UPWARD_SPEED) {
                liftMotor.set(speed);
            } else {
                liftMotor.set(Parameters.LIFT_MOTOR_MAX_UPWARD_SPEED);
            }
        } else if (liftState == LiftState.DOWN && lowerProximitySensor.getVoltage() > Parameters.PROXIMITY_SENSOR_THRESHOLD
                && pdp.getCurrent(Parameters.LIFT_MOTOR_PDP_CHANNEL) < Parameters.LIFT_MOTOR_CURRENT_THRESHOLD) {
            upwardAccelerationCounter = 0;
            liftMotor.set(-Parameters.LIFT_MOTOR_MAX_DOWNWARD_SPEED);
        } else {
            liftState = LiftState.STOPPED;
            upwardAccelerationCounter = 0;
            liftMotor.set(0);
        }

        //New Power Cube Eating Mode - PAC MAN!
        if (pacManMode) {
            int visionTargetOffset = coprocessor.getPowerCubeCoord() - 160;
            double turnSpeed = ((double) visionTargetOffset) / 160;

            int followDistance = coprocessor.getRangedDistance();
            double followSpeed = ((double) followDistance) / 1500;
            if (Math.abs(followSpeed) > 0.6) {
                followSpeed = Math.copySign(0.6, followSpeed);
            }

            System.out.println(coprocessor.getPowerCubeCoord());
            if (followDistance < 320) { //Reached target... eat cube!
                robotDrive.stopMotor();
                clawSolenoid.set(DoubleSolenoid.Value.kReverse); //eat the cube
            } else if (coprocessor.getPowerCubeCoord() > 0 && coprocessor.getPowerCubeCoord() < 400) {
                System.out.println("Following at " + followSpeed + "; turning at " + turnSpeed);
                robotDrive.arcadeDrive(followSpeed, turnSpeed, false);
            } else {
                System.out.println("No target found, Stopping");
                robotDrive.stopMotor();
                pacManMode = false;
            }
        }
    }


    /**
     * Teleop button controls that don't change based on the control mode go here.
     * The button assignments will remain constant.
     */
    private void teleopOperatorControls(Joystick js) {
        if (js.getRawButton(1)) {
            System.out.println("Entering PAC MAN mode");
            System.out.println("wakawakawakawakawaka");
            liftState = LiftState.DOWN;
            clawSolenoid.set(DoubleSolenoid.Value.kForward); //open
            pacManMode = true;
        }

        if (js.getRawButton(2)) {
            System.out.println("Leaving PAC MAN mode");
            pacManMode = false;
        }

        if (js.getRawButtonPressed(3)) {
            shifterSolenoid.set(DoubleSolenoid.Value.kForward); //low
            inLowGear = true;
        }

        if (js.getRawButtonPressed(4)) {
            shifterSolenoid.set(DoubleSolenoid.Value.kReverse); //high
            inLowGear = false;
        }

        /*if (js.getRawAxis(5) < -0.2) {
            liftState = LiftState.UP;
        } else if (js.getRawAxis(5) > 0.2) {
            liftState = LiftState.DOWN;
        } else {
            liftState = LiftState.STOPPED;
        }*/ //TODO make manual and automatic lift controls interoperable with each other

        if (js.getRawButton(5) && upperProximitySensor.getVoltage() > Parameters.PROXIMITY_SENSOR_THRESHOLD) {
            liftState = LiftState.UP;
        }

        if (js.getRawButton(6) && lowerProximitySensor.getVoltage() > Parameters.PROXIMITY_SENSOR_THRESHOLD) {
            liftState = LiftState.DOWN;
        }

        if (js.getRawButtonPressed(7)) {
            clawSolenoid.set(DoubleSolenoid.Value.kReverse); //closed
        }

        if (js.getRawButtonPressed(8)) {
            clawSolenoid.set(DoubleSolenoid.Value.kForward); //open
        }

        if (js.getRawButtonPressed(9)) {
            imu.setYaw(0, 0);
        }
    }

    @Override
    public void disabledInit() {
        robotDrive.stopMotor();
        liftMotor.stopMotor();

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
        TURN_TO_ANGLE_AND_LIFT,
        MOVE_TO_POSITION,
        DALEK_MODE,
        BACKUP_TURN_TO_ZERO,
        BACKUP_DRIVE_FORWARD,
        DEPOSIT_CUBE,
        FINISHED
    }

    public enum LiftState {
        UP,
        DOWN,
        STOPPED
    }
}
