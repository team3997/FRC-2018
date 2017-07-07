package org.usfirst.frc.team3997.robot.controllers;

import org.usfirst.frc.team3997.robot.hardware.Ports;

import edu.wpi.first.wpilibj.DigitalOutput;

public class LightController {
	static final boolean HIGH = true;
	static final boolean LOW = false;
	private DigitalOutput pin1;
	private DigitalOutput pin2;
	private DigitalOutput pin3;
	private DigitalOutput pin4;
	
	public LightController() {
		pin1 = new DigitalOutput(Ports.LIGHTS_DIO_PORTS[0]);
		pin2 = new DigitalOutput(Ports.LIGHTS_DIO_PORTS[1]);
		pin3 = new DigitalOutput(Ports.LIGHTS_DIO_PORTS[2]);
	    pin4 = new DigitalOutput(Ports.LIGHTS_DIO_PORTS[3]);
	}
	public void setDisabledLights() {
		pin1.set(LOW);
		pin2.set(LOW);
		pin3.set(LOW);
		pin4.set(LOW);
	}

	public void setEnabledLights() {
		pin1.set(HIGH);
		pin2.set(LOW);
	    pin3.set(LOW);
	    pin4.set(LOW);
	}
	
	public void setErrorLights() {
	    pin1.set(HIGH);
	    pin2.set(HIGH);
	    pin3.set(HIGH);
	    pin4.set(LOW);
	}
	//was gearIntake
	public void setAutoLights() {
	    pin1.set(LOW);
	    pin2.set(HIGH);
	    pin3.set(HIGH);
	    pin4.set(LOW);
	}

}
