package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.Joystick;

public class ToggleButtonReader extends ButtonReader {
	private boolean currToggleState;
	
	public ToggleButtonReader(Joystick joy, int buttonNum) {
		super(joy, buttonNum);
		currToggleState = false;
	}
	public boolean getState() {
		if (wasJustReleased()) {
			currToggleState = !currToggleState;
		}
		return (currToggleState);
	}
}