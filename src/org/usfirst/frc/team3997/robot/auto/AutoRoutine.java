package org.usfirst.frc.team3997.robot.auto;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.actions.Action;
import org.usfirst.frc.team3997.robot.auto.actions.ArcadeStraightAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveDistanceAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveIntervalAction;
import org.usfirst.frc.team3997.robot.auto.actions.DriveRotateAction;

import org.usfirst.frc.team3997.robot.auto.actions.PathFollowerAction;
import org.usfirst.frc.team3997.robot.auto.actions.VisionAction;
import org.usfirst.frc.team3997.robot.auto.actions.WaitAction;

import edu.wpi.first.wpilibj.RobotState;
import jaci.pathfinder.Waypoint;

public abstract class AutoRoutine {
	public boolean m_active = false;
	public boolean teleop;
	
	public void driveInterval(MasterController controllers, double seconds, double y, double x) {
		runAction(new DriveIntervalAction(controllers, seconds, y, x));
	}

	public void driveDistanceStraight(MasterController controllers, double desired_distance, double maxSpeed,
			double timeout, boolean waitForTimeout) {
		runAction(new DriveDistanceAction(controllers, desired_distance, maxSpeed, timeout, waitForTimeout));

	}

	public void driveRotate(MasterController controllers, double angle, double maxSpeed, double timeout,
			boolean waitForTimeout) {
		runAction(new DriveRotateAction(controllers, angle, maxSpeed, timeout, waitForTimeout));
	}

	public void vsionSetpointX(MasterController controllers, int setpoint, double maxSpeed,
			double timeout, boolean waitForTimeout) {
		runAction(new VisionAction(controllers, setpoint, maxSpeed, timeout, waitForTimeout));

	}
	
	public void arcadeDistanceStraight(MasterController controllers,
			double distance, double maxSpeed, double timeout, double timeAfterHit) {
		runAction(new ArcadeStraightAction(controllers, distance, maxSpeed, timeout, timeAfterHit));
	}
	
	public void pathFollower(MasterController controllers, Waypoint points[], double timeout) {
		runAction(new PathFollowerAction(controllers, points, timeout));
	}
	
	public void waitTime(double seconds) {
		runAction(new WaitAction(seconds));
	}
	
	public boolean isActive() {
		return m_active;
	}
	
	public void run() {
		m_active = true;
		routine();
	}
	
	public void stop() {
		m_active = false;
	}
	
	public void runAction(Action action) {
		action.start();
		//TODO maybe add timer test this first
		while((isActive()) && !(action.isFinished()) && (AutoRoutineRunner.getTimer().get() <= 15) && RobotState.isAutonomous() && !RobotState.isDisabled()) {
			action.update();
		}
		action.finish();
	}
	
	public abstract void prestart(); 
	
	protected abstract void routine();
	
	
	
}
