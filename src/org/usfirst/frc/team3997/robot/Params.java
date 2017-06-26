package org.usfirst.frc.team3997.robot;

public class Params {

	public static boolean SQUARE_DRIVE_AXIS_INPUT                    = true;
	public static boolean USE_ARCADE_DRIVE                     = true;

	public static double GLOBAL_Y_DRIVE_SPEED_MULTIPLIER          = 1.0;
	public static double GLOBAL_X_DRIVE_SPEED_MULTIPLIER          = 1.0;

	public static double X_ARCADE_DRIVE_OUT                       = 0.0;
	public static double X_ARCADE_DRIVE_LEFT_RIGHT				   = 6.25;
	public static double X_ARCADE_DRIVE_STRAIGHT				   = 6.25;

	public static final double HARDSET_DRIVE_SPEED_MAX			   = 0.95;

	//DRIVE PID PARAMS

	public static final double DRIVE_Y_PID_VALUES[]               = {1.0, 0.0, 0.0}; // P, I, D
	public static final double DRIVE_Y_PID_SCALE_VALUES[] 		   = {0.125, 1.0, 1.0}; //P, I, D

	public static final double DRIVE_Y_PID_TOLERANCE              = 0.5;
	public static final int DRIVE_Y_PID_SAMPLES_AVERAGE           = 1;

	public static final double DRIVE_X_PID_VALUES[]               = {0.0, 0.0, 0.0}; // P, I, D
	public static final double DRIVE_X_PID_SCALE_VALUES[] 		   = {1.0, 1.0, 1.0}; //P, I, D

	public static final int DRIVE_X_PID_TOLERANCE                 = 10;
	public static final int DRIVE_X_PID_SAMPLES_AVERAGE           = 10;


	//PINI UPDATES
	/*public static double PINI_P;
	public static double PINI_D;
	public static double PINI_I;*/

	//[DEBUGGING]

	//[DRIVE_PID]
	public static final double drive_p = 0.4;
	public static final double drive_i = 0.0;
	public static final double drive_d = 0.05;

	//[NEW_DRIVE_PID]
	public static final double new_drive_p = 0.4;
	public static final double new_drive_i = 0.0;
	public static final double new_drive_d = 0.1;


	//[CAMERA]
	public static final double h_low = 0;
	public static final double h_high = 180;
	public static final double s_low = 0;
	public static final double s_high = 255;
	public static final double v_low = 0;
	public static final double v_high = 255;

}
