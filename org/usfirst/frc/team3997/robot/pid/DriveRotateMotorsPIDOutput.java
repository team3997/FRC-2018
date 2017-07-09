package org.usfirst.frc.team3997.robot.pid;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;

public class DriveRotateMotorsPIDOutput implements PIDOutput{
	private RobotDrive drive;
	public DriveRotateMotorsPIDOutput(RobotDrive drive) {
		this.drive = drive;
	}
	@Override
	public void pidWrite(double output) {
		drive.arcadeDrive(0.0, output, false);
	}
	
	
}
