package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.RobotModel;
import edu.wpi.first.wpilibj.PIDOutput;

public class WheelsPIDOutput implements PIDOutput {

	private RobotModel.Wheels wheels;
	private RobotModel robot;
	private double loopOutput;
	public WheelsPIDOutput(RobotModel.Wheels wheels, RobotModel robot) {
		this.wheels = wheels;
		this.robot = robot;
	}
	@Override
	public void pidWrite(double output) {
		loopOutput = output;
		robot.setWheelSpeed(wheels, output);
	}
	
	double getPIDLoopOutput() {
		return loopOutput;
	}


}
