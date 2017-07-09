package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class DriveEncodersPIDSource implements PIDSource {
	private RobotModel robot;
	public DriveEncodersPIDSource(RobotModel robot) {
		this.robot = robot;
	}
	
	@Override
	public PIDSourceType getPIDSourceType() {
		return robot.leftDriveEncoder.getPIDSourceType();
		//TODO Add right encoder somehow
	}
	@Override
	public double pidGet() {
		switch (getPIDSourceType()) {
	    case kDisplacement:
	      return getAverageDistance();
	    case kRate:
	      return 0.0;
	    default:
	      return 0.0;
	  }
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		robot.leftDriveEncoder.setPIDSourceType(PIDSourceType.kRate);
		robot.rightDriveEncoder.setPIDSourceType(PIDSourceType.kRate);		
	}
	
	public double getAverageDistance() {
		return ((robot.leftDriveEncoder.getDistance()) + (robot.rightDriveEncoder.getDistance())) / 2.0;	
	}

}
