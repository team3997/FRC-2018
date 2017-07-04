package org.usfirst.frc.team3997.robot;

public class Params {

	// Params.h: Preferences for the robot
	public static boolean SQUARE_DRIVE_AXIS_INPUT = true;
	public static boolean USE_ARCADE_DRIVE = true;

	public static double GLOBAL_Y_DRIVE_SPEED_MULTIPLIER = 1.0;
	public static double GLOBAL_X_DRIVE_SPEED_MULTIPLIER = 1.0;

	public static double X_ARCADE_DRIVE_OUT = 0.0;
	public static double X_ARCADE_DRIVE_LEFT_RIGHT = 6.25;
	public static double X_ARCADE_DRIVE_STRAIGHT = 6.25;

	public static final double HARDSET_DRIVE_SPEED_MAX = 0.95;

	public static final double CLIMBER_HARDSET_MOTOR_SPEED = 1.0;

	// GEAR HOLDER PARAMS
	public static final double GEAR_WHEELS_RESTING_MOTOR_SPEED = 0.0;
	public static final double GEAR_WHEELS_ACTIVE_MOTOR_SPEED = 0.75;
	public static final double GEAR_WHEELS_OUT_MOTOR_SPEED = 0.35; // 0.575
	public static final double GEAR_TILTER_MAX_MOTOR_SPEED = 0.4;

	public static final double GEAR_POT_MAX_DOWN_UP[] = { 0.56, 0.45 };
	public static final double GEAR_POT_UP_POSITION = 0.425;
	public static final double GEAR_POT_RAMP_POSITION = 0.399;
	public static final double GEAR_POT_FORCE_DOWN_THRESHOLD = 0.464;
	public static final double GEAR_POT_FORCE_DOWN_SPEED = 0.15;

	public static final double GEAR_POT_FORCE_REST_THRESHOLD = 0.396;

	// DRIVE PID PARAMS

	public static final double DRIVE_Y_PID_VALUES[] = { 1.0, 0.0, 0.0 }; // P,
																			// I,
																			// D
	public static final double DRIVE_Y_PID_SCALE_VALUES[] = { 0.125, 1.0, 1.0 }; // P,
																					// I,
																					// D

	public static final double DRIVE_Y_PID_TOLERANCE = 0.5;
	public static final int DRIVE_Y_PID_SAMPLES_AVERAGE = 1;

	public static final double DRIVE_X_PID_VALUES[] = { 0.0, 0.0, 0.0 }; // P,
																			// I,
																			// D
	public static final double DRIVE_X_PID_SCALE_VALUES[] = { 1.0, 1.0, 1.0 }; // P,
																				// I,
																				// D

	public static final int DRIVE_X_PID_TOLERANCE = 10;
	public static final int DRIVE_X_PID_SAMPLES_AVERAGE = 10;

	// PINI UPDATES
	/*
	 * public static double PINI_P; public static double PINI_D; public static
	 * double PINI_I;
	 */

	// [DEBUGGING]

	// [DRIVE_PID]
	public static final double drive_p = 0.4;
	public static final double drive_i = 0.0;
	public static final double drive_d = 0.05;

	// [NEW_DRIVE_PID]
	public static final double new_drive_p = 0.4;
	public static final double new_drive_i = 0.0;
	public static final double new_drive_d = 0.1;

	// [GEAR_PID]
	public static final double gear_p = 12.5;
	public static final double gear_i = 0.000;
	public static final double gear_d = 9.0;

	public static final double gear_ramp_p = 12.5;
	public static final double gear_ramp_i = 0.000;
	public static final double gear_ramp_d = 2.5;

	// [GEAR_DOWN_PID]
	public static final double gear_down_p = 0.0;
	public static final double gear_down_i = 0.0;
	public static final double gear_down_d = 0.0;

	// [CAMERA]
	public static final double h_low = 0;
	public static final double h_high = 180;
	public static final double s_low = 0;
	public static final double s_high = 255;
	public static final double v_low = 0;
	public static final double v_high = 255;
	
	//[MOTION PROFILING] meters
	public static final double wheel_diameter = 0.1016;
	public static final double maximum_velocity = 1; //TODO add maximum velocity in m/s
	public static final double maximum_acceleration = 1; //TODO add maximum velocity in m/s/s
	public static final double maximum_jerk = 1; //TODO add maximum velocity in a/s

}
