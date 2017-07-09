package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;

public class TurnRoutine extends AutoRoutine {
	private MasterController controllers;
	public TurnRoutine(MasterController controllers) {
		this.controllers = controllers;
	}

	@Override
	public void prestart() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void routine() {
		driveRotate(controllers, 90, .5, 1.6, true);
	}

}
