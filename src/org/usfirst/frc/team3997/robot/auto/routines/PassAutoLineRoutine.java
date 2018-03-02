package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.controllers.MotionController;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class PassAutoLineRoutine extends AutoRoutine {
	private MasterController controllers;
	Waypoint[] point = new Waypoint[] {
			new Waypoint(0,10,0), 
			new Waypoint(0,0,0)
	};
	Trajectory traj;
	public PassAutoLineRoutine(MasterController controllers) {
		this.controllers = controllers;
		
	}

	@Override
	public void prestart() {
		// TODO Auto-generated method stub
		traj = MotionController.generateTrajectory(point);
	}

	@Override
	protected void routine() {
		pathFollower(controllers, traj, 30);
	}
}
