package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.auto.actions.WaitAction;
import org.usfirst.frc.team3997.robot.feed.DashboardVariables;

public class CustomDistanceRoutine extends AutoRoutine {
	MasterController controllers;
	public CustomDistanceRoutine(MasterController controllers) {
		this.controllers = controllers;
	}
	@Override
	public void prestart() {
		
	}

	@Override
	protected void routine() {
		controllers.getGearController().gearRest();
		/*arcadeDistanceStraight(controllers, DashboardVariables.firstAutoDistance, .8, DashboardVariables.firstAutoDistanceTimeout, .4);
		driveRotate(controllers, DashboardVariables.nextAutoAngle, .5, DashboardVariables.nextAutoAngleTimeout, false);*/
		if(DashboardVariables.gearDown) {
			controllers.getGearController().gearDown();
		}
		/*arcadeDistanceStraight(controllers, DashboardVariables.lastAutoDistance, .8, DashboardVariables.lastAutoDistanceTimeout, .4);*/
		
	}

}