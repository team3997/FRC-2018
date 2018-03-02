package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.PIDOutput;

public class AbsoluteEncoderPIDOutput implements PIDOutput{
	
	RobotModel robot;
	
	public AbsoluteEncoderPIDOutput(RobotModel robot) {
		this.robot = robot;
	}

	@Override
	public void pidWrite(double output) {
		robot.moveArm(output);
	}

}