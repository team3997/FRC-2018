package org.usfirst.frc.team3997.robot.feed;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class DashboardVariables {

	public static double firstAutoDistance = 0.0;
	public static double firstAutoDistanceTimeout = 0.0;

	public static double nextAutoAngle = 0.0;
	public static double nextAutoAngleTimeout = 0.0;

	public static double lastAutoDistance = 0.0;
	public static double lastAutoDistanceTimeout = 0.0;

	
	public static double DRIVE_P = 0.0;
	public static double DRIVE_I = 0.0;
	public static double DRIVE_D = 0.0;
	
	public static double GEAR_P = 0.0;
	public static double GEAR_I = 0.0;
	public static double GEAR_D = 0.0;
	
	public static double GEAR_RAMP_P = 0.0;
	public static double GEAR_RAMP_I = 0.0;
	public static double GEAR_RAMP_D = 0.0;
	
	public static boolean gearDown = false;
	public static boolean gearRamp = true;
	
	public static double max_speed = 1;
	
	//Paths
	public static double redLeftGearWaypoints[][] = {{-4, -1, Pathfinder.d2r(-45)}, {-2, -2, 0}, {0, 0, 0}};
	public static Waypoint redLeftGear[];
	

}
