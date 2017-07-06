package org.usfirst.frc.team3997.robot.controllers;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import jaci.pathfinder.*;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class MotionController {
	private RobotModel robot;
	private Waypoint[] points;
	public Trajectory.Config config;
	public Trajectory trajectory;
	public TankModifier modifier;
	public EncoderFollower left;
	public EncoderFollower right;
	private boolean isEnabled;
	public MotionController(RobotModel robot) {
		this.robot = robot;
		isEnabled = false;
	}

	public void setUp(Waypoint[] points) {
		this.points = points;
		config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, Params.maximum_velocity,
				Params.maximum_acceleration, Params.maximum_jerk);
		trajectory = Pathfinder.generate(points, config);

		// TODO find distance between front and rear axles of a vehicle
		modifier = new TankModifier(trajectory).modify(0.5);
		left = new EncoderFollower(modifier.getLeftTrajectory());
		right = new EncoderFollower(modifier.getRightTrajectory());
	}

	public void enable() {
		// TODO get max velocity
		// TODO find ticks_per_revolution
		// .1016 meters = 4 inch wheel diameter
		isEnabled = true;
		robot.resetGyro();
		left.configureEncoder(robot.leftDriveEncoder.get(), 100, .1016);
		right.configureEncoder(robot.rightDriveEncoder.get(), 100, .1016);
		left.configurePIDVA(1.0, 0.0, 0.0, (1 / Params.maximum_velocity), 0);
		right.configurePIDVA(1.0, 0.0, 0.0, (1 / Params.maximum_velocity), 0);

		right.configureEncoder(robot.leftDriveEncoder.get(), 100, .1016);
		right.configureEncoder(robot.rightDriveEncoder.get(), 100, .1016);
		right.configurePIDVA(1.0, 0.0, 0.0, (1 / Params.maximum_velocity), 0);
		right.configurePIDVA(1.0, 0.0, 0.0, (1 / Params.maximum_velocity), 0);
		
	}
	// TODO Put this in control loop
	public void update() {
		if(isEnabled) {
			double l = left.calculate(robot.leftDriveEncoder.get());
			double r = left.calculate(robot.rightDriveEncoder.get());
			
			double gyro_heading = robot.getAngle();
			double desired_heading = Pathfinder.r2d(left.getHeading());
			double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
			double turn = 0.8 * (-1.0 / 80) * angleDifference;
	
			robot.setLeftMotors(l + turn);
			robot.setRightMotors(r - turn);
		}
	}
	public void disable() {
		isEnabled = false;
		robot.setLeftMotors(0);
		robot.setRightMotors(0);
	}

}
