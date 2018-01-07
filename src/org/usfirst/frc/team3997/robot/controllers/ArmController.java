package org.usfirst.frc.team3997.robot.controllers;

import org.usfirst.frc.team3997.robot.hardware.RemoteControl;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl.Joysticks;

import org.usfirst.frc.team3997.robot.hardware.RobotModel;

public class ArmController {
	private RobotModel robot;
	private RemoteControl humanControl;
	
	private ClimberState m_stateVal;
	private ClimberState nextState;
	
	public enum ClimberState {
		kInitialize, kTeleop
	};
	public ArmController(RobotModel robot, RemoteControl humanControl) {
		this.robot = robot;
		this.humanControl = humanControl;
		
		m_stateVal = ClimberState.kInitialize;
		nextState = ClimberState.kInitialize;
	}
	
	public void reset() {
		m_stateVal = ClimberState.kInitialize;
	}
	
	public void update() {
		switch(m_stateVal) {
		case kInitialize:
			nextState = ClimberState.kTeleop;
			break;
		case kTeleop:
			//Climber Behavior
			if () {
				robot.moveArm();
			} else {
				robot.moveArm();
			}
			nextState = ClimberState.kTeleop;
			break;
		}
		m_stateVal = nextState;
	}

}