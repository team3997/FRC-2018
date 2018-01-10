package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.Joystick;

public class ButtonReader {
	private Joystick joystick;
	private int buttonNum;
	private boolean lastState;
	private boolean currState;

	public ButtonReader(Joystick joystick, int buttonNum) {
		this.joystick = joystick;
		this.buttonNum = buttonNum;
		currState = this.joystick.getRawButton(buttonNum);
		lastState = currState;
	}

	public void readValue() {
		lastState = currState;
		currState = joystick.getRawButton(buttonNum);
	}

	public boolean isDown() {
		return currState;
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

}


