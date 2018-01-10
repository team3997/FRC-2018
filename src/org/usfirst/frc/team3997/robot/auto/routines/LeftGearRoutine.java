package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;


import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class LeftGearRoutine extends AutoRoutine {
	private MasterController controllers;
	public LeftGearRoutine(MasterController controllers) {
		this.controllers = controllers;
	}

	@Override
	public void prestart() {

		
	}

	@Override
	protected void routine() {
		controllers.getGearController().gearPIDUp();
		
		/*pathFollower(controllers, new Waypoint[] {
			    new Waypoint(0, 98, Pathfinder.d2r(70)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
			    new Waypoint(15, 2, 70)                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
			                               // Waypoint @ x=0, y=0,   exit angle=0 radians
			}, 6.0);
			
			*
			*
			*
			*/

		arcadeDistanceStraight(controllers, 98.0, 0.6, 4.0, 0.4);
		driveRotate(controllers, 70, 0.6, 1.6, true);
		arcadeDistanceStraight(controllers, 25.0, 0.6, 4, 0.4);
		controllers.getGearController().gearDown();
		waitTime(0.05);
		driveDistanceStraight(controllers, -15.0, .95, 2, true);
		driveRotate(controllers, -70, 0.6, 1.6, true);
		arcadeDistanceStraight(controllers, 75.0, .95, 4.0, 0.4);

		//TODO controllers.getMotionController().enable();
	}

}
