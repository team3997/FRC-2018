package org.usfirst.frc.team3997.robot.controllers;

import org.usfirst.frc.team3997.robot.hardware.Ports;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LightController {

	public static final int DisabledLights = 1;
	public static final int EnabledLights = 2;
	public static final int ErrorLights = 3;
	public static final int AutoLights = 4;
	public static final int ArmLights = 5;
	public static final int IntakeLights = 6;
	public static final int ClimbLights = 7; // change that later

	// create byte array to send and the I2C object "lights"
	public byte[] toSend = new byte[1];
	I2C arduino;

	public LightController(I2C arduino) {
		this.arduino = arduino;
	}
	
	public void setLights(int mode) {
		toSend[0]=(byte)mode;
	
	if(arduino.addressOnly()) {
		arduino.transaction(toSend, 1, null, 0);
		SmartDashboard.putString("ARDUINO_STATUS", "FOUND");

	} else {
		SmartDashboard.putString("ARDUINO_STATUS", "NOT FOUND");
	}
	}

}
