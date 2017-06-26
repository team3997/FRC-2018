package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArcadeStraightPIDOutput implements PIDOutput {
	private RobotDrive drive;
	private RobotModel robot;
	private double loopOutput;
	private double kPencoder; //0.625
	public ArcadeStraightPIDOutput(RobotDrive drive, RobotModel robot) {
		this.drive = drive;
		this.robot = robot;
		kPencoder = 0.625;
	}
	@Override
	public void pidWrite(double output) {
		loopOutput = output;
		drive.arcadeDrive(output, robot.getEncoderError() * kPencoder, false);
	    SmartDashboard.putNumber("ArcadeCORRECTION", robot.getEncoderError() * kPencoder);
	    SmartDashboard.putNumber("auton_EncoderError", robot.getEncoderError());
	}
	
	double getPIDLoopOutput() {
		return loopOutput;
	}


}
