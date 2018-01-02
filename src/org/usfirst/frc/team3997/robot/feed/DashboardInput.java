package org.usfirst.frc.team3997.robot.feed;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import sun.rmi.runtime.NewThreadAction;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.networktables.TableEntryListener;
/*
 * 
 * dolub NOTIFY_IMMEDIATE = 0x01;
int NOTIFY_LOCAL = 0x02;
int NOTIFY_NEW = 0x04;
int NOTIFY_DELETE = 0x08;
int NOTIFY_UPDATE = 0x10;
*/
public class DashboardInput {
	
	Preferences preferences;
	public DashboardInput() {

		
		DashboardVariables.firstAutoDistance = preferences.getDouble("First Auto Distance", 0);

		DashboardVariables.nextAutoAngle = preferences.getDouble("Next Auto Angle", 0);
		
		DashboardVariables.lastAutoDistance = preferences.getDouble("Second Auto Distance", 0);
		
		
		DashboardVariables.DRIVE_P = preferences.getDouble("Drive P Value", 0);
		DashboardVariables.DRIVE_I = preferences.getDouble("Drive I Value", 0);
		DashboardVariables.DRIVE_D = preferences.getDouble("Drive D Value", 0);

		
		DashboardVariables.max_speed = preferences.getDouble("Max Speed", 1);

	}
	
	public void updateInput() {
		

		DashboardVariables.firstAutoDistance = preferences.getDouble("First Auto Distance", 0);

		DashboardVariables.nextAutoAngle = preferences.getDouble("Next Auto Angle", 0);
		
		DashboardVariables.lastAutoDistance = preferences.getDouble("Second Auto Distance", 0);
		
		
		DashboardVariables.DRIVE_P = preferences.getDouble("Drive P Value", 0);
		DashboardVariables.DRIVE_I = preferences.getDouble("Drive I Value", 0);
		DashboardVariables.DRIVE_D = preferences.getDouble("Drive D Value", 0);

		
		DashboardVariables.max_speed = preferences.getDouble("Max Speed", 1);
		
	}
	

}
