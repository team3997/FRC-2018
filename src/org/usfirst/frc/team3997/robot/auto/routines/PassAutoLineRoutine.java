package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;

public class PassAutoLineRoutine extends AutoRoutine {
	private MasterController controllers;
	public PassAutoLineRoutine(MasterController controllers) {
		this.controllers = controllers;
	}

	@Override
	public void prestart() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void routine() {
		driveDistanceStraight(controllers, 100, .8, 4.0, false);
	}
}
