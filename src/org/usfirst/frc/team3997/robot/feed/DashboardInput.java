package org.usfirst.frc.team3997.robot.feed;

import edu.wpi.first.wpilibj.Preferences;


public class DashboardInput {
	
	Preferences preferences;
	public DashboardInput() {

		
		DashboardVariables.firstAutoDistance = preferences.getDouble("First Auto Distance", 0);

		DashboardVariables.nextAutoAngle = preferences.getDouble("Next Auto Angle", 0);
		
		DashboardVariables.lastAutoDistance = preferences.getDouble("Second Auto Distance", 0);
		
		
		DashboardVariables.DRIVE_P = preferences.getDouble("Drive P Value", 0);
		DashboardVariables.DRIVE_I = preferences.getDouble("Drive I Value", 0);
		DashboardVariables.DRIVE_D = preferences.getDouble("Drive D Value", 0);

		DashboardVariables.ARM_P = preferences.getDouble("ARM P Value", 0);
		DashboardVariables.ARM_I = preferences.getDouble("ARM I Value", 0);
		DashboardVariables.ARM_D = preferences.getDouble("ARM D Value", 0);
		DashboardVariables.ARM_F = preferences.getDouble("ARM F Value", 0);

		DashboardVariables.CLIMB_ARM_ANGLE = preferences.getDouble("CLIMB_ARM_ANGLE", 150);
		DashboardVariables.SCALE_ARM_ANGLE = preferences.getDouble("SCALE_ARM_ANGLE", 120);
		DashboardVariables.SWITCH_ARM_ANGLE = preferences.getDouble("SWITCH_ARM_ANGLE", 90);
		DashboardVariables.FEED_ARM_ANGLE = preferences.getDouble("FEED_ARM_ANGLE", 10);

		
		DashboardVariables.max_speed = preferences.getDouble("Max Speed", 1);

	}
	
	public void updateInput() {
		

		DashboardVariables.firstAutoDistance = preferences.getDouble("First Auto Distance", 0);

		DashboardVariables.nextAutoAngle = preferences.getDouble("Next Auto Angle", 0);
		
		DashboardVariables.lastAutoDistance = preferences.getDouble("Second Auto Distance", 0);
		
		DashboardVariables.ARM_P = preferences.getDouble("ARM P Value", 0);
		DashboardVariables.ARM_I = preferences.getDouble("ARM I Value", 0);
		DashboardVariables.ARM_D = preferences.getDouble("ARM D Value", 0);
		DashboardVariables.ARM_F = preferences.getDouble("ARM F Value", 0);

		DashboardVariables.max_speed = preferences.getDouble("Max Speed", 1);
		
	}
	

}
