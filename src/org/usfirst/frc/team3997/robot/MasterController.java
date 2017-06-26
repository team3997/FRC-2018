package org.usfirst.frc.team3997.robot;
import org.usfirst.frc.team3997.robot.controllers.*;
import org.usfirst.frc.team3997.robot.hardware.*;



public class MasterController {
	private VisionController vision;
	private DriveController driveTrain;
	private LightController lights;
	private RobotModel robot;
	
	public MasterController(DriveController driveTrain, RobotModel robot, VisionController vision, LightController lights) {
		this.vision = vision;
		this.driveTrain = driveTrain;
		this.robot = robot;
		this.lights = lights;
	}
	
	public VisionController getVisionController() {
		return vision;
	}
	
	public RobotModel getRobotModel() {
		return robot;
	}
	
	public DriveController getDriveController() {
		return driveTrain;
	}
	
	public LightController getLightController() {
		return lights;
	}
}
