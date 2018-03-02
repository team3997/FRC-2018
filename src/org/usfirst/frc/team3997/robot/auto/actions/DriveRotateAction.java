package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.controllers.DriveController;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveRotateAction extends Action{
	private DriveController driveTrain;
	private RobotModel robot;
	
	private double distance;
	private double timeout;
	private double maxSpeed;
	private double P, I, D;
	private double leftEncoderStartDistance, rightEncoderStartDistance;
	
	private boolean reachedSetpoint, waitForTimeout;
	
	public DriveRotateAction(MasterController controllers, double angle, double maxSpeed, double timeout, boolean waitForTimeout) {
		this.driveTrain = controllers.getDriveController();
		this.distance = (angle * 40.0) / (180.0);
		this.timeout = timeout;
		this.robot = controllers.getRobotModel();
		this.maxSpeed = maxSpeed;
		this.waitForTimeout = waitForTimeout;
		
		reachedSetpoint = false;
		leftEncoderStartDistance = 0.0; 
		rightEncoderStartDistance = 0.0;
		
		P = 0.3;
		//TODO Might Need to change pID values
		I = 0.0;
		D = 0.0;
		
		SmartDashboard.putNumber("DRIVE_PID_P", P);
		SmartDashboard.putNumber("DRIVE_PID_I", I);
		SmartDashboard.putNumber("DRIVE_PID_D", D);

	}
	@Override
	public boolean isFinished() {
		if((Timer.getFPGATimestamp() >= start_time + timeout) && !(reachedSetpoint)) {
			
		}
		if(waitForTimeout) 
			return (Timer.getFPGATimestamp() >= start_time + timeout);
		else
			return (Timer.getFPGATimestamp() >= start_time + timeout) || reachedSetpoint;
	}

	@Override
	public void update() {
		robot.updateGyro();
		if(driveTrain.leftPID.onTarget() && driveTrain.rightPID.onTarget()) {
			reachedSetpoint = true;
		} else {
			reachedSetpoint = false;
		}
	}

	@Override
	public void finish() {
		driveTrain.leftPID.disable();
		driveTrain.rightPID.disable();
		driveTrain.stop();
	}

	@Override
	public void start() {
		start_time = Timer.getFPGATimestamp();
		
		robot.leftDriveEncoder.reset();
		robot.rightDriveEncoder.reset();
		
		leftEncoderStartDistance = robot.leftDriveEncoder.getDistance();
		rightEncoderStartDistance = robot.rightDriveEncoder.getDistance();
		
		driveTrain.leftPID.setOutputRange(-maxSpeed, maxSpeed);
		driveTrain.leftPID.setPID(P, I, D);
		driveTrain.leftPID.setSetpoint(distance + leftEncoderStartDistance);
		
		driveTrain.rightPID.setOutputRange(-maxSpeed, maxSpeed);
		driveTrain.rightPID.setPID(P, I, D);
		driveTrain.rightPID.setSetpoint(distance + rightEncoderStartDistance);
		
		driveTrain.leftPID.enable();
		driveTrain.rightPID.enable();
	}

}
