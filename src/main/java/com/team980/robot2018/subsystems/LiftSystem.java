package com.team980.robot2018.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team980.robot2018.Parameters;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class LiftSystem {

    private WPI_TalonSRX liftMotor;
    private Encoder liftEncoder;

    private LiftPosition position;
    private LiftState state;

    private int upwardAccelerationCounter = 0;

    private PowerDistributionPanel pdp;
    private NetworkTable table;

    public LiftSystem(PowerDistributionPanel pdp, NetworkTable table) {
        liftMotor = new WPI_TalonSRX(Parameters.LIFT_MOTOR_CAN_ID);
        liftMotor.setName("Lift System", "Lift Motor");

        liftEncoder = new Encoder(Parameters.LIFT_ENCODER_DIO_CHANNEL_A, Parameters.LIFT_ENCODER_DIO_CHANNEL_B, Parameters.INVERT_LIFT_ENCODER, CounterBase.EncodingType.k4X);
        liftEncoder.setName("Lift System", "Lift Encoder");

        position = LiftPosition.SCALE; //this should ALWAYS be true
        state = LiftState.STOPPED; //We don't want to move the lift right now!

        this.pdp = pdp;
        this.table = table;
    }

    public LiftPosition getPosition() {
        return position;
    }

    public void setPosition(LiftPosition position) {
        this.position = position;

        if (liftEncoder.getRaw() < position.getDistance()) {
            state = LiftState.MOVING_UP;
        } else if (liftEncoder.getRaw() > position.getDistance()) {
            state = LiftState.MOVING_DOWN;
        } else {
            state = LiftState.STOPPED;
        }
    }

    public LiftState getState() {
        return state;
    }

    public boolean hasReachedPosition(LiftPosition position) {
        switch (state) {
            case MOVING_UP:
                return liftEncoder.getRaw() > position.getDistance();
            case MOVING_DOWN:
                return liftEncoder.getRaw() < position.getDistance();
            default:
                return true;
        }
    }

    public void updateData() {
        table.getSubTable("Lift System").getEntry("Lift Position").setString(position.name());
        table.getSubTable("Lift System").getEntry("Raw Encoder Position").setNumber(liftEncoder.getRaw());
        table.getSubTable("Lift System").getEntry("Lift State").setString(state.name());
        table.getSubTable("Lift System").getEntry("Lift Motor Current").setNumber(pdp.getCurrent(Parameters.LIFT_MOTOR_PDP_CHANNEL));
    }

    public void resetEncoder() {
        liftEncoder.reset();
    }

    public void operateLift(Joystick js) { //Manual override
        if (Math.abs(js.getRawAxis(1)) > 0.2) {
            state = LiftState.STOPPED;
            liftMotor.set(-js.getRawAxis(1) * Parameters.LIFT_MOTOR_MAX_MANUAL_SPEED);
        } else { //Automatic
            operateLift();
        }
    }

    public void operateLift() {
        if (state == LiftState.MOVING_UP && !hasReachedPosition(position)
                && pdp.getCurrent(Parameters.LIFT_MOTOR_PDP_CHANNEL) < Parameters.LIFT_MOTOR_CURRENT_THRESHOLD) {
            upwardAccelerationCounter++;
            double speed = Parameters.LIFT_MOTOR_MIN_UPWARD_SPEED + (Parameters.LIFT_MOTOR_UPWARD_ACCELERATION * upwardAccelerationCounter);
            if (speed < Parameters.LIFT_MOTOR_MAX_UPWARD_SPEED) {
                liftMotor.set(speed);
            } else {
                liftMotor.set(Parameters.LIFT_MOTOR_MAX_UPWARD_SPEED);
            }
        } else if (state == LiftState.MOVING_DOWN && !hasReachedPosition(position)
                && pdp.getCurrent(Parameters.LIFT_MOTOR_PDP_CHANNEL) < Parameters.LIFT_MOTOR_CURRENT_THRESHOLD) {
            upwardAccelerationCounter = 0;
            liftMotor.set(-Parameters.LIFT_MOTOR_MAX_DOWNWARD_SPEED);
        } else {
            state = LiftState.STOPPED;
            upwardAccelerationCounter = 0;
            liftMotor.set(0);
        }
    }

    public void disable() {
        liftMotor.stopMotor();
        state = LiftState.STOPPED;
    }

    public enum LiftPosition {
        BOTTOM(Parameters.LIFT_ENCODER_BOTTOM_DISTANCE),
        SWITCH(Parameters.LIFT_ENCODER_SWITCH_DISTANCE),
        SCALE(Parameters.LIFT_ENCODER_SCALE_DISTANCE);

        private double distance;

        LiftPosition(double distance) {
            this.distance = distance;
        }

        public double getDistance() {
            return distance;
        }
    }

    public enum LiftState {
        MOVING_UP,
        MOVING_DOWN,
        STOPPED
    }
}