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
		controllers.getGearController().gearPIDUp();
		//SmartDashboard.putNumber("customRoutine", 1);
		//arcadeDistanceStraight(controllers, DashboardVariables.firstAutoDistance, .2, DashboardVariables.firstAutoDistanceTimeout, 10);
		//arcadeDistanceStraight(controllers, 5, .2, 10, 10);
		//arcadeDistance is messed up4
		SmartDashboard.putNumber("customROutine", 0);

		driveDistanceStraight(controllers, DashboardVariables.firstAutoDistance, .7, DashboardVariables.firstAutoDistanceTimeout, true);

		Timer.delay(5);
		//SmartDashboard.putNumber("customRoutine", 2);

		driveRotate(controllers, DashboardVariables.nextAutoAngle, .5, DashboardVariables.nextAutoAngleTimeout, false);
		
		/*if(DashboardVariables.gearDown) {
			controllers.getGearController().gearDown();
		}*/
		//Timer.delay(5);
		//SmartDashboard.putNumber("customRoutine", 3);


		driveDistanceStraight(controllers, DashboardVariables.lastAutoDistance, .2, DashboardVariables.lastAutoDistanceTimeout, true);
		SmartDashboard.putNumber("customROutine", 100);
		Timer.delay(5);
		
	}

}