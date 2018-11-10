/*
 *  MIT License
 *
 *  Copyright (c) 2018 FRC Team 980 ThunderBots
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

package com.team980.robot2018.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team980.robot2018.Parameters;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;

public class LiftSystem {

    private WPI_TalonSRX liftMotor;
    private Encoder liftEncoder;

    private LiftPosition position;
    private LiftState state;

    private int upwardAccelerationCounter = 0;

    private NetworkTable table;

    public LiftSystem(NetworkTable table) {
        liftMotor = new WPI_TalonSRX(Parameters.LIFT_MOTOR_CAN_ID);
        liftMotor.setName("Lift System", "Lift Motor");

        liftEncoder = new Encoder(Parameters.LIFT_ENCODER_DIO_CHANNEL_A, Parameters.LIFT_ENCODER_DIO_CHANNEL_B, Parameters.INVERT_LIFT_ENCODER, CounterBase.EncodingType.k4X);
        liftEncoder.setName("Lift System", "Lift Encoder");

        position = LiftPosition.BOTTOM; //this should ALWAYS be true
        state = LiftState.STOPPED; //We don't want to move the lift right now!

        this.table = table;
    }

    public Encoder getEncoder() {
        return liftEncoder;
    }

    public LiftPosition getPosition() {
        return position;
    }

    public void setPosition(LiftPosition position) {
        this.position = position;
        state = LiftState.MOVING_TO_POSITION;

        if (position == LiftPosition.HOLD_CURRENT) {
            LiftPosition.heldPosition = liftEncoder.getRaw();
        } else {
            LiftPosition.heldPosition = -1;
        }
    }

    public LiftState getState() {
        return state;
    }

    public void updateData() {
        table.getSubTable("Lift System").getEntry("Lift Position").setString(position.name());
        table.getSubTable("Lift System").getEntry("Raw Encoder Position").setNumber(liftEncoder.getRaw());
        table.getSubTable("Lift System").getEntry("Held Position").setNumber(LiftPosition.heldPosition);
        table.getSubTable("Lift System").getEntry("Lift State").setString(state.name());
        table.getSubTable("Lift System").getEntry("Lift Motor Current").setNumber(liftMotor.getOutputCurrent());
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
        if (state == LiftState.MOVING_TO_POSITION && liftMotor.getOutputCurrent() < Parameters.LIFT_MOTOR_CURRENT_THRESHOLD) {
            if (liftEncoder.getRaw() < position.getDistance()
                    && liftEncoder.getRaw() < Parameters.LIFT_ENCODER_TOP_DISTANCE) {
                upwardAccelerationCounter++;
                double speed = Parameters.LIFT_MOTOR_MIN_UPWARD_SPEED + (Parameters.LIFT_MOTOR_UPWARD_ACCELERATION * upwardAccelerationCounter);
                if (speed < Parameters.LIFT_MOTOR_MAX_UPWARD_SPEED) {
                    liftMotor.set(speed);
                } else {
                    liftMotor.set(Parameters.LIFT_MOTOR_MAX_UPWARD_SPEED);
                }
            } else if (liftEncoder.getRaw() > position.getDistance() + Parameters.LIFT_SYSTEM_POSITION_DEADBAND
                    && liftEncoder.getRaw() > Parameters.LIFT_ENCODER_BOTTOM_DISTANCE) {
                upwardAccelerationCounter = 0;
                liftMotor.set(-Parameters.LIFT_MOTOR_MAX_DOWNWARD_SPEED);
            } else {
                upwardAccelerationCounter = 0;
                liftMotor.set(0);
            }
        } else {
            upwardAccelerationCounter = 0;
            liftMotor.set(0);
        }
    }

    public void disable() {
        liftMotor.disable();
        state = LiftState.STOPPED;
    }

    public enum LiftPosition {
        BOTTOM(Parameters.LIFT_ENCODER_BOTTOM_DISTANCE),
        AUTO(Parameters.LIFT_ENCODER_AUTO_DISTANCE),
        SWITCH(Parameters.LIFT_ENCODER_SWITCH_DISTANCE),
        SCALE(Parameters.LIFT_ENCODER_SCALE_DISTANCE),

        /**
         * Used for custom hold-at button
         */
        HOLD_CURRENT(-1);

        private double distance;

        private static double heldPosition;

        LiftPosition(double distance) {
            this.distance = distance;
        }

        public double getDistance() {
            if (this == HOLD_CURRENT) {
                return heldPosition;
            } else {
                return distance;
            }
        }
    }

    public enum LiftState {
        MOVING_TO_POSITION,
        STOPPED
    }
}
