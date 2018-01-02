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
	Preferences preferences;
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
		
		double defaultPID[] = {0.4, 0.0, 0.1};
		double newPID[] = prefs.getNumberArray("DRIVE_PID", defaultPID);
		DashboardVariables.DRIVE_P = newPID[0];
		DashboardVariables.DRIVE_I = newPID[0];
		DashboardVariables.DRIVE_D = newPID[0];

		
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
		
		double defaultPID[] = {0.4, 0.0, 0.1};
		double newPID[] = prefs.getNumberArray("DRIVE_PID", defaultPID);
		DashboardVariables.DRIVE_P = newPID[0];
		DashboardVariables.DRIVE_I = newPID[0];
		DashboardVariables.DRIVE_D = newPID[0];


		DashboardVariables.max_speed = prefs.getNumber("MAX_SPEED", 1.0);
		
	}
	

}
