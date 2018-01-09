package org.usfirst.frc.team3997.robot;
import org.usfirst.frc.team3997.robot.controllers.*;
import org.usfirst.frc.team3997.robot.hardware.*;

public class MasterController {
	private VisionController vision;
	private DriveController driveTrain;
	private LightController lights;
	private RobotModel robot;
	private MotionController motion;
	private ArmController armController;
	
	public MasterController(DriveController driveTrain, RobotModel robot, MotionController motion, VisionController vision, LightController lights, ArmController armController) {
		this.vision = vision;
		this.driveTrain = driveTrain;
		this.robot = robot;
		this.motion = motion;
		this.lights = lights;
		this.armController = armController;
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
	
	public MotionController getMotionController() {
		return motion;
	}
	
	public ArmController getArmController() {
		return armController;
	}
		

}
