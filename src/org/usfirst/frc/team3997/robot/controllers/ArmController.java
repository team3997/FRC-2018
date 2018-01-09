package org.usfirst.frc.team3997.robot.controllers;

import java.util.ResourceBundle.Control;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl.Joysticks;

import org.usfirst.frc.team3997.robot.hardware.RobotModel;
import org.usfirst.frc.team3997.robot.pid.AbsoluteEncoderPIDOutput;
import org.usfirst.frc.team3997.robot.pid.AbsoluteEncoderPIDSource;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;


public class ArmController {
	private RobotModel robot;
	private RemoteControl humanControl;
	private PIDController armPIDController;
	private PIDOutput armPIDOutput;
	private PIDSource armPIDSource;
	
	private ArmState m_stateVal;
	private ArmState nextState;
	private boolean toggleArmManual;

	public enum ArmState {
		kInitialize, kTeleop
	};

	public ArmController(RobotModel robot, RemoteControl humanControl) {
		this.robot = robot;
		this.humanControl = humanControl;
		
		toggleArmManual = false;
		
		armPIDSource = new AbsoluteEncoderPIDSource(robot.armEncoder);
		armPIDOutput = new AbsoluteEncoderPIDOutput(robot);
		armPIDController = new PIDController(0, 0, 0, 0, armPIDSource, armPIDOutput);
		
		armPIDController.setOutputRange(-1, 1);
		armPIDController.setAbsoluteTolerance(0.25);
		armPIDController.setSetpoint(Params.ARM_REST_SETPOINT);
		armPIDController.disable();
		
		
		m_stateVal = ArmState.kInitialize;
		nextState = ArmState.kInitialize;
	}

	public void reset() {
		m_stateVal = ArmState.kInitialize;
	}

	// ???
	public void update() {
		switch(m_stateVal) {
		case kInitialize:
			
			
			armPIDController.setPID(Params.arm_p,Params.arm_i,Params.arm_d,Params.arm_f);
			armPIDController.setOutputRange(-1, 1);
			armPIDController.setSetpoint(Params.ARM_REST_SETPOINT);
			toggleArmManual = false;
			
			nextState = ArmState.kTeleop;
			break;
		case kTeleop:
			if(humanControl.toggleManualArmDesired())
				toggleArmManual = !toggleArmManual;
			//Arm Behavior
			if (toggleArmManual) {
				robot.moveArm(humanControl.getJoystickValue(Joysticks.kOperatorJoy, RemoteControl.Axes.kRY));
			} else {
				if (armPIDController.isEnabled()) {
					armPIDController.disable();
				}
				
				if(humanControl.getClimbArmDesired()) {
					goToClimbPosition();
				} else if(humanControl.getScaleArmDesired()) {
					goToScalePosition();
				} else if(humanControl.getSwitchArmDesired()) {
					goToSwitchPosition();
				} else if(humanControl.getFeedArmDesired()) {
					goToFeedPosition();
				}
			}
			
			/* The else {} section above will just disable the PID controller every single loop if not in manual control.
			Have you just not added in logic for the functions below yet?  -Aaron  */
			
			nextState = ArmState.kTeleop;
			break;
		}
		m_stateVal = nextState;
	}
	
	public void goToSwitchPosition() {
		armPIDController.setPID(Params.arm_p,Params.arm_i,Params.arm_d,Params.arm_f);
		armPIDController.setOutputRange(-1, 1);
		armPIDController.setSetpoint(Params.ARM_SWITCH_SETPOINT);
		armPIDController.enable();
	}
	
	public void goToScalePosition() {
		armPIDController.setPID(Params.arm_p,Params.arm_i,Params.arm_d,Params.arm_f);
		armPIDController.setOutputRange(-1, 1);
		armPIDController.setSetpoint(Params.ARM_SCALE_SETPOINT);
		armPIDController.enable();
	}
	
	public void goToClimbPosition() {
		armPIDController.setPID(Params.arm_p,Params.arm_i,Params.arm_d,Params.arm_f);
		armPIDController.setOutputRange(-1, 1);
		armPIDController.setSetpoint(Params.ARM_CLIMB_SETPOINT);
		armPIDController.enable();
	}
	
	public void goToFeedPosition() {
		armPIDController.setPID(Params.arm_p,Params.arm_i,Params.arm_d,Params.arm_f);
		armPIDController.setOutputRange(-1, 1);
		armPIDController.setSetpoint(Params.ARM_FEED_SETPOINT);
		armPIDController.enable();
	}
	
	
	public void stop() {
		armPIDController.disable();
		robot.moveArm(0);
	}

}