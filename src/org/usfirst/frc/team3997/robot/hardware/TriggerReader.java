package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.Joystick;

public class TriggerReader {
	private Joystick joystick;
	private int triggerAxis;
	private boolean lastState;
	private boolean currState;

	public TriggerReader(Joystick joystick, int triggerAxis) {
		this.joystick = joystick;
		this.triggerAxis = triggerAxis;
		
		if(joystick.getRawAxis(triggerAxis) > 0.8) {
			currState = true;
		} else {
			currState = false;
		}
		lastState = currState;
	}
	
	public void readValue() {
		lastState = currState;
		if(joystick.getRawAxis(triggerAxis) > 0.8) {
			currState = true;
		} else {
			currState = false;
		}
	}
	
	public boolean wasJustPressed() {
		return (lastState == false && currState == true);
	}
	
	public boolean wasJustReleased() {
		return (lastState == true && currState == false);
	}
	
	public boolean stateJustChanged() {
		return (lastState != currState);
	}
	
	public boolean isDown() {
		return currState;
	}

}
