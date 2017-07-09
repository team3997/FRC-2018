package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;

public class DriveThreeSecRoutine extends AutoRoutine {
	private MasterController controllers;
	public DriveThreeSecRoutine(MasterController controllers) {
		this.controllers = controllers;
	}

	@Override
	public void prestart() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void routine() {
		driveInterval(controllers, 3, .5, 0);
	}

}
