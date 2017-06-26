package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.controllers.VisionController;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;
/*
 * TODO Learn how to make camera a pid source
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;*/

public class VisionPIDSource {
	private VisionController vision;
	private RobotModel robot;
	public VisionPIDSource(VisionController vision, RobotModel robot) {
		this.vision = vision;
		this.robot = robot;
	}
	
/*	@Override
	public PIDSourceType getPIDSourceType() {
		return robot.camera.pu;
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
	}*/

}
