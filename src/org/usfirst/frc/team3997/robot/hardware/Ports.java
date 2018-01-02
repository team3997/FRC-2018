package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.I2C.Port;

public class Ports {
	public static final int GEAR_POT_OFFSET                        = 0;
	// ***************** PWM PORTS *****************
	public static final int LEFT_DRIVE_MOTOR_A_PWM_PORT            = 0; //front left
	public static final int LEFT_DRIVE_MOTOR_B_PWM_PORT            = 1; //back left
	public static final int RIGHT_DRIVE_MOTOR_A_PWM_PORT           = 2; //front right
	public static final int RIGHT_DRIVE_MOTOR_B_PWM_PORT           = 3; //back right
	public static final int CLIMBER_MOTOR_PWM_PORT                 = 6;
	public static final int GEAR_INTAKE_MOTOR_PWM_PORT             = 5;
	public static final int GEAR_TILTER_MOTOR_PWM_PORT             = 4;
	public static final int CLIMBER_LOCKER_SERVO_PWM_PORT          = 7;

	// ***************** PDP CHANNELS *****************
	public static final int LEFT_DRIVE_MOTOR_A_PDP_CHAN           = 12;
	public static final int LEFT_DRIVE_MOTOR_B_PDP_CHAN           = 13;
	public static final int RIGHT_DRIVE_MOTOR_A_PDP_CHAN          = 14;
	public static final int RIGHT_DRIVE_MOTOR_B_PDP_CHAN          = 15;
	public static final int CLIMBER_MOTOR_PDP_CHAN                = 16;

	// ***************** DIGITAL I/O PORTS *****************
	public static final int LIGHTS_DIO_PORTS[]                    = {2, 3, 4, 5};
	public static final int LEFT_DRIVE_ENCODER_PORTS[]            = {9, 8};
	public static final int RIGHT_DRIVE_ENCODER_PORTS[]           = {7, 6};

	// ******************* ANALOG IN PORTS*******************
	public static final int GEAR_POT                              = 0;

	// ***************** MISC *****************

	// ***************** SOLENOID PORTS *****************

	// ***************** JOYSTICK USB PORTS *****************
	//MAKE SURE JOYSTICKS ARE SET TO "D" position on back

	public static final boolean USING_WIN_DRIVER_STATION             = true;
	public static final int DRIVER_JOY_USB_PORT                   = 0;
	public static final int OPERATOR_JOY_USB_PORT                 = 1;
	// ***************** BUTTONS *****************

	//Controller button ports

	public static final int DRIVE_DIRECTION_BUTTON_PORT           = 9;
	public static final int SHOOTER_RUN_BUTTON_PORT               = 1;
	public static final int GYRO_PORT = 0;

}
