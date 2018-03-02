package org.usfirst.frc.team3997.robot;

import org.usfirst.frc.team3997.robot.controllers.*;
import org.usfirst.frc.team3997.robot.hardware.*;

/**
 * Getter and setter class that packages important classes for autonomous. e.g
 * {@code master.getRobotModel()} returns the RobotModel class passed in
 * Robot.java
 * 
 * @author Peter I. Chacko, Eric Warner, John Sullivan Jr, Jake Boothby, Elliot
 *         Friedlander
 * 
 * @param driveTrain
 *            Gets a DriveController object
 * @param robot
 *            Gets a RobotModel object
 * @param motion
 *            Gets a MotionController object
 * @param vision
 *            Gets a VisionController object
 * @param lights
 *            Gets a LightConroller object
 * 
 */
public class MasterController {
	private VisionController vision;
	private DriveController driveTrain;
	private LightController lights;
	private RobotModel robot;
	private MotionController motion;
	private ArmController armController;

	/**
	 * 
	 * Takes private classes and sets to the ones assigned in Robot.java
	 * 
	 * @param driveTrain
	 *            Gets a DriveController object
	 * @param robot
	 *            Gets a RobotModel object
	 * @param motion
	 *            Gets a MotionController object
	 * @param vision
	 *            Gets a VisionController object
	 * @param lights
	 *            Gets a LightConroller object
	 */
	public MasterController(DriveController driveTrain, RobotModel robot, MotionController motion,
			VisionController vision, LightController lights, ArmController armController) {

		this.vision = vision;
		this.driveTrain = driveTrain;
		this.robot = robot;
		this.motion = motion;
		this.lights = lights;
		this.armController = armController;
	}

	/**
	 * Returns VisionController object initialization in Robot.java
	 *
	 **/
	public VisionController getVisionController() {
		return vision;
	}

	/**
	 * Returns RobotModel object initialization in Robot.java
	 *
	 **/
	public RobotModel getRobotModel() {
		return robot;
	}

	/**
	 * Returns DriveController object initialization in Robot.java
	 *
	 **/
	public DriveController getDriveController() {
		return driveTrain;
	}

	/**
	 * Returns LightController object initialization in Robot.java
	 *
	 **/
	public LightController getLightController() {
		return lights;
	}

	/**
	 * Returns MotionController object initialization in Robot.java
	 *
	 **/
	public MotionController getMotionController() {
		return motion;
	}

	public ArmController getArmController() {
		return armController;
	}


}
