package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.controllers.MotionController;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class PathFollowerAction extends Action{
	private MotionController motion;
	private RobotModel robot;
	private double timeout;
	public PathFollowerAction(MasterController controllers, Trajectory trajectory, double timeout) {
		this.motion = controllers.getMotionController();
		this.robot = controllers.getRobotModel();
		this.timeout = timeout;
		
		this.motion.setUp(trajectory);
	}
	@Override
	public boolean isFinished() {
		return ((motion.left.isFinished()) && (motion.right.isFinished())) || (Timer.getFPGATimestamp() >= start_time + timeout);
	}

	@Override
	public void update() {		
		motion.update();
	}

	@Override
	public void finish() {
		motion.disable();
	}

	@Override
	public void start() {
		start_time = Timer.getFPGATimestamp();
		robot.leftDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		robot.rightDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		robot.resetGyro();
		robot.leftDriveEncoder.reset();
		robot.rightDriveEncoder.reset();
		motion.enable();

	}

}
