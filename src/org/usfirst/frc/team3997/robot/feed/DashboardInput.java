package org.usfirst.frc.team3997.robot.feed;

import edu.wpi.first.wpilibj.Preferences;

public class DashboardInput {
	private Preferences prefs;
	public DashboardInput() {
		prefs = Preferences.getInstance();
		
		DashboardVariables.firstAutoDistance = prefs.getDouble("First Auto Distance", 0.0);
		DashboardVariables.nextAutoAngle = prefs.getDouble("Next Auto Angle", 0.0);
		DashboardVariables.lastAutoDistance = prefs.getDouble("Last Auto Distance", 0.0);

		DashboardVariables.P = prefs.getDouble("P", 0.0);
		DashboardVariables.I = prefs.getDouble("I", 0.0);
		DashboardVariables.D = prefs.getDouble("D", 0.0);
	}
	
	public void updateInput() {
		prefs = Preferences.getInstance();
		
		DashboardVariables.firstAutoDistance = prefs.getDouble("First Auto Distance", 0.0);
		DashboardVariables.nextAutoAngle = prefs.getDouble("Next Auto Angle", 0.0);
		DashboardVariables.lastAutoDistance = prefs.getDouble("Last Auto Distance", 0.0);

		DashboardVariables.P = prefs.getDouble("P", 0.0);
		DashboardVariables.I = prefs.getDouble("I", 0.0);
		DashboardVariables.D = prefs.getDouble("D", 0.0);
	}
	

}
