package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;

public class CenterGearRoutine extends AutoRoutine {
	private MasterController controllers;

	public CenterGearRoutine(MasterController controllers) {
		this.controllers = controllers;
	}

	@Override
	public void prestart() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void routine() {
		controllers.getGearController().gearPIDUp();
		arcadeDistanceStraight(controllers, 75.0, 0.5, 4.0, 0.4);
		controllers.getGearController().gearDown();
		driveDistanceStraight(controllers, -5.0, .7, 1.5, true);
	}

}
