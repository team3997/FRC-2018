package org.usfirst.frc.team3997.robot.feed;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;

public class DashboardInput {
	private NetworkTable prefs;
	ITableListener m_listener;
	Preferences pref;
	public DashboardInput() {
		m_listener = new ITableListener() {
		    @Override
		    public void valueChanged(ITable table, String key, Object value, boolean isNew) {
		      // unused
		    }

		    @Override
		    public void valueChangedEx(ITable table, String key, Object value, int flags) {
		      table.setPersistent(key);
		    }
		  };
		prefs = NetworkTable.getTable("Preferences");
		prefs.addTableListenerEx(m_listener, ITable.NOTIFY_NEW | ITable.NOTIFY_IMMEDIATE);
	    HAL.report(tResourceType.kResourceType_Preferences, 0);
		double defaultAutoDistances[][] = {{5.0, 8.0},{30.0, 4.0},{4.0, 5.0}};
		double customAutoDistances[][] = {prefs.getNumberArray("First Auto Distance", defaultAutoDistances[0]), prefs.getNumberArray("First Auto Distance", defaultAutoDistances[1]), prefs.getNumberArray("First Auto Distance", defaultAutoDistances[2])};
		DashboardVariables.firstAutoDistance = customAutoDistances[0][0];
		DashboardVariables.firstAutoDistanceTimeout = customAutoDistances[0][1];

		DashboardVariables.nextAutoAngle = customAutoDistances[1][0];
		DashboardVariables.nextAutoAngleTimeout = customAutoDistances[1][1];
		
		DashboardVariables.lastAutoDistance = customAutoDistances[2][0];
		DashboardVariables.lastAutoDistanceTimeout = customAutoDistances[2][1];
		
		double defaultPID[][] = {{0.4, 0.0, 0.1}, {12.5, 0, 9.5}, {12.5, 0, 2.5}};
		double newPID[][] = {prefs.getNumberArray("DRIVE_PID", defaultPID[0]), prefs.getNumberArray("GEAR_PID", defaultPID[1]), prefs.getNumberArray("GEAR_RAMP_PID", defaultPID[2])};
		DashboardVariables.DRIVE_P = newPID[0][0];
		DashboardVariables.DRIVE_I = newPID[0][1];
		DashboardVariables.DRIVE_D = newPID[0][2];
		
		DashboardVariables.GEAR_P = newPID[1][0];
		DashboardVariables.GEAR_I = newPID[1][1];
		DashboardVariables.GEAR_D = newPID[1][2];
		
		DashboardVariables.GEAR_RAMP_P = newPID[2][0];
		DashboardVariables.GEAR_RAMP_I = newPID[2][1];
		DashboardVariables.GEAR_RAMP_D = newPID[2][2];

		DashboardVariables.gearRamp = prefs.getBoolean("GoToGearRamp", false);
		DashboardVariables.gearDown = prefs.getBoolean("GoToGearDown", false);
		
		DashboardVariables.max_speed = prefs.getNumber("MAX_SPEED", 1);

	}
	
	public void updateInput() {
		
		double defaultAutoDistances[][] = {{5.0, 8.0},{30.0, 4.0},{4.0, 5.0}};
		double customAutoDistances[][] = {prefs.getNumberArray("First Auto Distance", defaultAutoDistances[0]), prefs.getNumberArray("First Auto Distance", defaultAutoDistances[1]), prefs.getNumberArray("First Auto Distance", defaultAutoDistances[2])};
		DashboardVariables.firstAutoDistance = customAutoDistances[0][0];
		DashboardVariables.firstAutoDistanceTimeout = customAutoDistances[0][1];

		DashboardVariables.nextAutoAngle = customAutoDistances[1][0];
		DashboardVariables.nextAutoAngleTimeout = customAutoDistances[1][1];
		
		DashboardVariables.lastAutoDistance = customAutoDistances[2][0];
		DashboardVariables.lastAutoDistanceTimeout = customAutoDistances[2][1];
		
		double defaultPID[][] = {{0.4, 0.0, 0.1}, {12.5, 0, 9.5}, {12.5, 0, 2.5}};
		double newPID[][] = {prefs.getNumberArray("DRIVE_PID", defaultPID[0]), prefs.getNumberArray("GEAR_PID", defaultPID[1]), prefs.getNumberArray("GEAR_RAMP_PID", defaultPID[2])};
		DashboardVariables.DRIVE_P = newPID[0][0];
		DashboardVariables.DRIVE_I = newPID[0][1];
		DashboardVariables.DRIVE_D = newPID[0][2];
		
		DashboardVariables.GEAR_P = newPID[1][0];
		DashboardVariables.GEAR_I = newPID[1][1];
		DashboardVariables.GEAR_D = newPID[1][2];
		
		DashboardVariables.GEAR_RAMP_P = newPID[2][0];
		DashboardVariables.GEAR_RAMP_I = newPID[2][1];
		DashboardVariables.GEAR_RAMP_D = newPID[2][2];

		DashboardVariables.gearRamp = prefs.getBoolean("GoToGearRamp", false);
		DashboardVariables.gearDown = prefs.getBoolean("GoToGearDown", false);

		
	}
	

}
