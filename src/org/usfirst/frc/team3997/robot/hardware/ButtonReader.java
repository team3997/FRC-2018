package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.Joystick;
/** Debounces and reads controller values 
 * @category hardware
 * @author Peter I. Chacko, Eric Warner, John Sullivan Jr, Jake Boothby, Elliot
 *         Friedlander
 *         **/
public class ButtonReader {
	private Joystick joystick;
	private int buttonNum;
	/** Last state of the button **/
	private boolean lastState;
	/** Current state of the button **/
	private boolean currState;

	/** Initializes controller variables 
	 * @param joystick The controller joystick
	 * @param buttonNum The controller button number **/
	public ButtonReader(Joystick joystick, int buttonNum) {
		this.joystick = joystick;
		this.buttonNum = buttonNum;
		currState = this.joystick.getRawButton(buttonNum);
		lastState = currState;
	}
	/** Reads button value **/
	public void readValue() {
		lastState = currState;
		currState = joystick.getRawButton(buttonNum);
	}
	/** Checks if the button is down **/
	public boolean isDown() {
		return currState;
	}
	/** Checks if the button was just pressed **/
	public boolean wasJustPressed() {
		return (lastState == false && currState == true);
	}
	/** Checks if the button was just released **/
	public boolean wasJustReleased() {
		return (lastState == true && currState == false);
	}
	/** Checks if the button state has changed **/
	public boolean stateJustChanged() {
		return (lastState != currState);
	}

}


