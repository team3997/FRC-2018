package org.usfirst.frc.team3997.robot.controllers;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.hardware.Ports;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;
import org.usfirst.frc.team3997.robot.pid.AnalogPIDOutput;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearController {

	private RobotModel robot;
	private RemoteControl humanControl;

	boolean wasDown, wasUp, wasRest, wasRamp, toggleManual;

	GearState m_stateVal, nextState;

	public PIDOutput gearTilterPIDOutput;
	public PIDController gearTilterPID;

	public enum GearState {
		kInitialize, kTelop
	};

	public GearController(RobotModel robot, RemoteControl humanControl) {
		this.robot = robot;
		this.humanControl = humanControl;

		robot.gearPot.setPIDSourceType(PIDSourceType.kDisplacement);
		gearTilterPIDOutput = new AnalogPIDOutput(robot);
		gearTilterPID = new PIDController(0, 0, 0, robot.gearPot, gearTilterPIDOutput);

		gearTilterPID.setInputRange(-1.0, 1.0);
		gearTilterPID.setPID(0, 0, 0);
		gearTilterPID.setSetpoint(Params.GEAR_POT_UP_POSITION);
		gearTilterPID.disable();

		wasRest = false;
		wasRamp = false;
		wasRest = true;
		toggleManual = false;
		wasUp = false;
		m_stateVal = GearState.kInitialize;
		nextState = GearState.kInitialize;

	}

	public void reset() {
		m_stateVal = GearState.kInitialize;
	}

	public void update() {
		switch (m_stateVal) {
		case kInitialize:
			gearTilterPID.disable();
			wasDown = false;
			wasRest = true;
			wasRamp = false;
			wasUp = false;
			gearTilterPID.setPID(Params.gear_p, Params.gear_i, Params.gear_d);
			gearTilterPID.setOutputRange(-0.7, 0.7);
			gearTilterPID.setSetpoint(Params.GEAR_POT_UP_POSITION);
			toggleManual = false;
			nextState = GearState.kTelop;
			break;
		case kTelop:
			if (humanControl.getManualGearDesired())
				toggleManual = !toggleManual;

			// Pot Mode
			if (toggleManual == false) {
				if (humanControl.getGearTilterDownDesired()) {
					gearDown();
				} else if (humanControl.getGearTilterUpDesired()) {
					gearPIDRamp();
				} else if (humanControl.getGearIntakeDesired()) {
					gearIntake();
				} else {
					gearRest();
				}

				if (humanControl.getGearWheelIntakeDesired()) {
					robot.setGearIntakeSpeed(Params.GEAR_WHEELS_ACTIVE_MOTOR_SPEED);
				} else if (humanControl.getGearWheelOuttakeDesired()) {
					robot.setGearIntakeSpeed(-Params.GEAR_WHEELS_ACTIVE_MOTOR_SPEED);
				} else if (humanControl.getGearTilterRampDesired()) {
					robot.setGearIntakeSpeed(Params.GEAR_WHEELS_ACTIVE_MOTOR_SPEED);
				} else if (humanControl.getGearIntakeDesired()) {
					robot.setGearIntakeSpeed(Params.GEAR_WHEELS_ACTIVE_MOTOR_SPEED);
				} else {
					robot.setGearIntakeSpeed(Params.GEAR_WHEELS_RESTING_MOTOR_SPEED);
				}
			} else {
				softDisablePID();
				if (Math.abs(humanControl.getJoystickValue(RemoteControl.Joysticks.kOperatorJoy,
						RemoteControl.Axes.kLY)) < 0.1) {
					robot.setGearTilterSpeed(0.0);
				} else {
					robot.setGearTilterSpeed(
							humanControl.getJoystickValue(RemoteControl.Joysticks.kOperatorJoy, RemoteControl.Axes.kLY)
									* 0.65);
				}
				if (humanControl.getGearWheelIntakeDesired()) {
					robot.setGearIntakeSpeed(Params.GEAR_WHEELS_ACTIVE_MOTOR_SPEED);
				} else if (humanControl.getGearWheelOuttakeDesired()) {
					robot.setGearIntakeSpeed(-Params.GEAR_WHEELS_ACTIVE_MOTOR_SPEED);
				} else {
					robot.setGearIntakeSpeed(Params.GEAR_WHEELS_RESTING_MOTOR_SPEED);
				}
			}

			nextState = GearState.kTelop;
			break;
		}

		m_stateVal = nextState;
	}

	public void gearDown() {
		wasDown = true;
		wasRamp = false;
		wasRest = false;
		wasUp = false;
		softDisablePID();
		if (robot.getGearPotReading() <= Params.GEAR_POT_FORCE_DOWN_THRESHOLD) {
			robot.setGearTilterSpeed(0.15);
		} else {
			robot.setGearTilterSpeed(0.0);
		}
		SmartDashboard.putString("GEAR_STATE", "DOWN");
	}

	public void gearRest() {
		wasDown = false;
		wasRamp = false;
		wasUp = false;
		wasRest = true;
		softDisablePID();
		if (robot.getGearPotReading() >= Params.GEAR_POT_FORCE_DOWN_THRESHOLD) {
			robot.setGearTilterSpeed(-0.3);
		} else {
			robot.setGearTilterSpeed(0.0);
		}
		SmartDashboard.putString("GEAR_STATE", "Rest");
	}

	public void gearPIDRamp() {
		wasDown = false;
		wasRamp = false;
		wasUp = false;
		wasRest = true;
		gearTilterPID.setPID(Params.gear_p, Params.gear_i, Params.gear_d);
		gearTilterPID.setSetpoint(Params.GEAR_POT_RAMP_POSITION);
		gearTilterPID.setOutputRange(-0.4, 0.4);
		gearTilterPID.enable();
		SmartDashboard.putString("GEAR_STATE", "Ramp");
	}

	public void gearPIDUp() {
		if (wasDown)
			gearTilterPID.reset();
		wasDown = false;
		wasRamp = false;
		wasUp = true;
		wasRest = false;
		gearTilterPID.setPID(Params.gear_p, Params.gear_i, Params.gear_d);
		gearTilterPID.setSetpoint(Params.GEAR_POT_UP_POSITION);
		gearTilterPID.setOutputRange(-0.65, 0.65);
		gearTilterPID.enable();
		SmartDashboard.putString("GEAR_STATE", "PID:UP");
	}
	
	public void gearIntake() {
		wasDown = true;
	    wasRamp = false;
	    wasRest = false;
	    wasUp = false;
	    softDisablePID();
	    if(robot.getGearPotReading() <= Params.GEAR_POT_FORCE_DOWN_THRESHOLD) {
	    	robot.setGearTilterSpeed(0.4);
	    } else {
	    	robot.setGearTilterSpeed(0.4);
	    }
	    SmartDashboard.putString("GEAR_STATE", "DOWN");
	}

	public void softDisablePID() {
		if(gearTilterPID.isEnabled()) {
			gearTilterPID.disable();
		}
	}

}