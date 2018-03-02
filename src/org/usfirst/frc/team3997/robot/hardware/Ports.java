package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.I2C.Port;

public class Ports {
	public static final int GEAR_POT_OFFSET                        = 0;
	// ***************** PWM PORTS *****************

	public static final int CLIMBER_MOTOR_PWM_PORT                 = 6;
	public static final int LEFT_ARM_MOTOR_PWM_PORT                = 8;
	public static final int RIGHT_ARM_MOTOR_PWM_PORT               = 3;
	public static final int LEFT_INTAKE_MOTOR_PWM_PORT             = 9;
	public static final int RIGHT_INTAKE_MOTOR_PWM_PORT            = 2;
	//public static final int CLIMBER_LOCKER_SERVO_PWM_PORT          = 7;
	public static final int LEFT_DRIVE_MOTOR_A_PWM_PORT            = 6; //front left
	public static final int LEFT_DRIVE_MOTOR_B_PWM_PORT            = 7; //back left
	public static final int RIGHT_DRIVE_MOTOR_A_PWM_PORT           = 5; //front right
	public static final int RIGHT_DRIVE_MOTOR_B_PWM_PORT           = 4; //back right

	// ***************** PDP CHANNELS *****************
	public static final int LEFT_DRIVE_MOTOR_A_PDP_CHAN           = 12;
	public static final int LEFT_DRIVE_MOTOR_B_PDP_CHAN           = 13;
	public static final int RIGHT_DRIVE_MOTOR_A_PDP_CHAN          = 14;
	public static final int RIGHT_DRIVE_MOTOR_B_PDP_CHAN          = 15;
	public static final int CLIMBER_MOTOR_PDP_CHAN                = 16;

	// ***************** DIGITAL I/O PORTS *****************
	public static final int LIGHTS_DIO_PORTS[]                    = {2, 3, 4, 5};
	public static final int LEFT_DRIVE_ENCODER_PORTS[]            = {9, 8};
	//Top Ports
	public static final int RIGHT_DRIVE_ENCODER_PORTS[]           = {7, 6};

	// ******************* ANALOG IN PORTS*******************
	public static final int ARM_ENCODER                              = 0;
	

	// ***************** MISC *****************

	// ***************** SOLENOID PORTS *****************
	public static final int COMPRESSOR_MODULE = 9;
	public static final int[] SOLENOID_MODULE = {0, 1};
	public static final int[] SOLENOID_CHANNEL = {0, 1, 2, 3};

	// ***************** JOYSTICK USB PORTS *****************
	//MAKE SURE JOYSTICKS ARE SET TO "D" position on back

	public static final boolean USING_WIN_DRIVER_STATION             = true;
	public static final int DRIVER_JOY_USB_PORT                   = 0;
	public static final int OPERATOR_JOY_USB_PORT                 = 1;
	// ***************** BUTTONS *****************

	//Controller button ports
	public static final Port gyroPort = Port.kOnboard;
	public static final int DRIVE_DIRECTION_BUTTON_PORT           = 9;
	public static final int SHOOTER_RUN_BUTTON_PORT               = 1;
	public static final int GYRO_PORT = 0;

}
