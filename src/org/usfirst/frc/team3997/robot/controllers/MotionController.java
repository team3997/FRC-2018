package org.usfirst.frc.team3997.robot.controllers;

import java.io.File;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import jaci.pathfinder.*;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Handles path planning and following
 * 
 * @category controllers
 * @author Peter I. Chacko, Eric Warner, John Sullivan Jr, Jake Boothby, Elliot
 *         Friedlander
 * 
 **/
public class MotionController {
	private RobotModel robot;
	/**
	 * A single waypoint used for Trajectory Generation.
	 *
	 * A Waypoint is a 'setpoint' that you wish for your trajectory to intersect.
	 * Waypoints are given an X, Y coordinate stating their location in space, and
	 * an exit angle that defines the heading the trajectory should point towards
	 * once this waypoint is reached. The angle is given in Radians
	 *
	 * @author Jaci
	 */
	private Waypoint[] points;
	/**
	 * The Trajectory Configuration outlines the rules to follow while generating
	 * the trajectory. This includes the method used for 'fitting' the spline, the
	 * amount of samples to use, the time difference and maximum values for the
	 * velocity, acceleration and jerk of the trajectory.
	 * 
	 * @author Jaci
	 */
	public Trajectory.Config config;
	/**
	 * The Trajectory object contains an array of Segments that represent the
	 * location, velocity, acceleration, jerk and heading of a particular point in
	 * the trajectory.
	 *
	 * Trajectories can be generated with the Pathfinder class
	 *
	 * @author Jaci
	 */
	public Trajectory trajectory;
	/**
	 * The Tank Modifier will take in a Source Trajectory and a Wheelbase Width and
	 * spit out a Trajectory for each side of the wheelbase. This is commonly used
	 * in robotics for robots which have a drive system similar to a 'tank', where
	 * individual parallel sides are driven independently
	 *
	 * The Source Trajectory is measured from the centre of the drive base. The
	 * modification will not modify the central trajectory
	 *
	 * @author Jaci
	 */
	public TankModifier modifier;
	/**
	 * The EncoderFollower is an object designed to follow a trajectory based on
	 * encoder input. This class can be used for Tank or Swerve drive
	 * implementations.
	 *
	 * @author Jaci
	 */
	public EncoderFollower left;
	/**
	 * The EncoderFollower is an object designed to follow a trajectory based on
	 * encoder input. This class can be used for Tank or Swerve drive
	 * implementations.
	 *
	 * @author Jaci
	 */
	public EncoderFollower right;
	private boolean isEnabled;

	/** Gets RobotModel object and sets boolean isEnabled to false **/
	public MotionController(RobotModel robot) {
		this.robot = robot;
		isEnabled = false;
	}
	/** Sets up config, trajectory, tank modifier, and encoderFollowers using an array of Waypoints **/
	public void setUp(Waypoint[] points) {
		this.points = points;
		config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05,
				Params.maximum_velocity, Params.maximum_acceleration, Params.maximum_jerk);
		trajectory = Pathfinder.generate(points, config);

		// TODO find distance between front and rear axles of a vehicle
		modifier = new TankModifier(trajectory).modify(0.5);
		left = new EncoderFollower(modifier.getLeftTrajectory());
		right = new EncoderFollower(modifier.getRightTrajectory());
	}

	/** Sets up tank modifier, and encoderFollowers using an already made trajectory **/

	public void setUp(Trajectory trajectoryInput) {

		trajectory = trajectoryInput;

		// TODO find distance between front and rear axles of a vehicle
		modifier = new TankModifier(trajectory).modify(0.5);
		left = new EncoderFollower(modifier.getLeftTrajectory());
		right = new EncoderFollower(modifier.getRightTrajectory());
	}
	/** Sets up trajectory, tank modifier, and encoderFollowers using a CSV file **/

	public void setUp(File trajectoryCSV) {

		trajectory = Pathfinder.readFromCSV(trajectoryCSV);

		// TODO find distance between front and rear axles of a vehicle
		modifier = new TankModifier(trajectory).modify(0.5);
		left = new EncoderFollower(modifier.getLeftTrajectory());
		right = new EncoderFollower(modifier.getRightTrajectory());
	}
	/** Static function that returns a trajectory given an array of waypoints **/
	public static Trajectory generateTrajectory(Waypoint[] points) {
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, Params.maximum_velocity, Params.maximum_acceleration,
				Params.maximum_jerk);
		return Pathfinder.generate(points, config);
	}
	/** Enables motion profiling **/
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

		robot.resetGyro();

	}

	// TODO Put this in control loop
	/** Runs motion profiling **/
	public void update() {
		
		robot.updateGyro();
		if (isEnabled) {
			double l = left.calculate(robot.leftDriveEncoder.get());
			double r = right.calculate(robot.rightDriveEncoder.get());

			double gyro_heading = robot.getAngle();

			double desired_heading = Pathfinder.r2d(left.getHeading());
			double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
			double turn = 0.8 * (-1.0 / 80) * angleDifference;
			robot.setLeftMotors(l + turn);
			robot.setRightMotors(r - turn);
		}
	}
	/** Stops motion profiling **/
	public void disable() {
		isEnabled = false;
		robot.setLeftMotors(0);
		robot.setRightMotors(0);
	}

}
