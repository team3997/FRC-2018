package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;

public class RightGearRoutine extends AutoRoutine {
	private MasterController controllers;

	public RightGearRoutine(MasterController controllers) {
		this.controllers = controllers;
	}

	@Override
	public void prestart() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void routine() {
		controllers.getGearController().gearPIDUp();
		arcadeDistanceStraight(controllers, 98.0, 0.6, 4.0, 0.4);
		driveRotate(controllers, -70, .6, 1.6, true);
		arcadeDistanceStraight(controllers, 30.0, 0.5, 2.5, 0.4);
		controllers.getGearController().gearDown();
		waitTime(0.05);
		driveDistanceStraight(controllers, -15.0, .95, 2, true);
		driveRotate(controllers, 70, 0.6, 1.6, true);
		arcadeDistanceStraight(controllers, 75.0, .95, 4.0, 0.4);
	}

}
