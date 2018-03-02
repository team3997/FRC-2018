package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.auto.actions.WaitAction;
import org.usfirst.frc.team3997.robot.feed.DashboardVariables;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
		driveDistanceStraight(controllers, DashboardVariables.firstAutoDistance, .8, 5, false);
		driveRotate(controllers, DashboardVariables.nextAutoAngle, .8, 5, false);
		driveDistanceStraight(controllers, DashboardVariables.lastAutoDistance, .8, 5, false);

	}

}