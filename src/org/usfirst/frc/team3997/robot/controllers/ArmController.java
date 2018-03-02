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
	
	// during 2017 competition the gear intake mech lost control and made the motors smoke
	// We needed a way to toggle between automatic mode and manual - this is what we are going for 
	// with toggleArmManual - no smoking!
	private boolean toggleArmManual;	// toggles the arm subsystem manual vs automatic using PID
	
	private boolean toggleCollapse;	  	// opens and closes the intake false is what ever the current position is

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
	}

	public void reset() {
		m_stateVal = ArmState.kInitialize;
		//?? should we also be disabling the arm?
	}

	// ???
	public void update() {
		switch(m_stateVal) {
			case kInitialize:
				armPIDController.setPID(Params.arm_p,Params.arm_i,Params.arm_d,Params.arm_f);
				armPIDController.setOutputRange(-1, 1);
				armPIDController.setAbsoluteTolerance(0.25);
				armPIDController.setSetpoint(Params.ARM_REST_SETPOINT);
				toggleArmManual = false; //
				toggleCollapse = false;  //
				m_stateVal = ArmState.kTeleop;
				break;
			case kTeleop:
				// check if we need to change into manual mode
				if(humanControl.toggleManualArmDesired())
					toggleArmManual = !toggleArmManual;
				
				// check if we need to change the intake collapse state
				if(humanControl.toggleCollapseIntake()) {
					toggleCollapse = !toggleCollapse;
				}
				
				// If we are in manual mode - read the arm values from the joystick
				// otherwise we will be reading values from the PID controller - which needs to be disabled first
				// to get a setpoint.
				if (toggleArmManual) {
					robot.moveArm(humanControl.getJoystickValue(Joysticks.kOperatorJoy, RemoteControl.Axes.kRY));
				} else {
					if (armPIDController.isEnabled()) {
						armPIDController.disable();
					}
				}
				
				// Decide what intake movement is needed (if any)
				// Should only be doing one - but in the case that the operator hits both buttons
				// perform the intake function
				// If neither button is pressed - stop the motors
				if(humanControl.getIntakeDesired()) {
					intakeBlock();
				}else if(humanControl.getOuttakeDesired()) {
					outtakeBlock();
				}
				else {
					stopIntake();
				}
				
				// Decide if intake should open or close (if any)
				// Should only be one - in the case that both buttons are pressed, close intake
				if(toggleCollapse) {
					robot.closeIntake();
				} else {
					robot.openIntake();
				}
					
				// Other arm movements and PID stuff	
				if(humanControl.getClimbArmDesired()) {
					goToClimbPosition();
				} else if(humanControl.getScaleArmDesired()) {
					goToScalePosition();
				} else if(humanControl.getSwitchArmDesired()) {
					goToSwitchPosition();
				} else if(humanControl.getFeedArmDesired()) {
					goToFeedPosition();
				}

				break;
		}
		
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

	public void intakeBlock() {
		robot.intakeBlock(0.7);
	}
	
	public void outtakeBlock() {
		robot.outtakeBlock(0.3);
	}
	
	public void stopIntake() {
		robot.intakeBlock(0);
	}
	
	public void stop() {
		armPIDController.disable();
		robot.moveArm(0);
	}

}