
package org.usfirst.frc.team3997.robot.hardware;

import org.usfirst.frc.team3997.robot.Params;
import edu.wpi.first.wpilibj.*;

public class ControlBoard extends RemoteControl {
	public ButtonReader arcadeDriveButton, tankDriveButton, driveBackButton, driveBackOtherButton;

	public TriggerReader slowDriveTier1Button, slowDriveTier2Button;

	private boolean tankDriveDesired, arcadeDriveDesired, slowDriveTier1Desired, slowDriveTier2Desired,
			driveBackDesired, driveBackOtherDesired;

	private double driverLeftJoyX, driverLeftJoyY, driverRightJoyX, driverRightJoyY;
	private double operatorLeftJoyX, operatorLeftJoyY, operatorRightJoyX, operatorRightJoyY;

	private Joystick driverJoy, operatorJoy;

	public ControlBoard() {
		driverJoy = new Joystick(Ports.DRIVER_JOY_USB_PORT);
		operatorJoy = new Joystick(Ports.OPERATOR_JOY_USB_PORT);

		if (Ports.USING_WIN_DRIVER_STATION) {
			arcadeDriveButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_START_BUTTON);
			tankDriveButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_BACK_BUTTON);
			driveBackButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_LEFT_BUMPER);
			driveBackOtherButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_RIGHT_BUMPER);
			slowDriveTier1Button = new TriggerReader(driverJoy, XInput.XINPUT_WIN_RIGHT_TRIGGER_AXIS);
			slowDriveTier2Button = new TriggerReader(driverJoy, XInput.XINPUT_WIN_LEFT_TRIGGER_AXIS);

		}

		driverLeftJoyX = 0;
		driverLeftJoyY = 0;
		driverRightJoyX = 0;
		driverRightJoyY = 0;

		// Drivetrain variables
		tankDriveDesired = !Params.USE_ARCADE_DRIVE;
		arcadeDriveDesired = Params.USE_ARCADE_DRIVE;

		slowDriveTier1Desired = false;
		slowDriveTier2Desired = false;
		driveBackDesired = false;
		driveBackOtherDesired = false;

	}

	public void readControls() {
		readAllButtons();
		if (Ports.USING_WIN_DRIVER_STATION) {
			driverLeftJoyX = driverJoy.getRawAxis(XInput.XINPUT_WIN_LEFT_X_AXIS);
			driverLeftJoyY = -driverJoy.getRawAxis(XInput.XINPUT_WIN_LEFT_Y_AXIS);
			driverRightJoyX = driverJoy.getRawAxis(XInput.XINPUT_WIN_RIGHT_X_AXIS);
			driverRightJoyY = -driverJoy.getRawAxis(XInput.XINPUT_WIN_RIGHT_Y_AXIS);

			operatorLeftJoyX = operatorJoy.getRawAxis(XInput.XINPUT_WIN_LEFT_X_AXIS);
			operatorLeftJoyY = -operatorJoy.getRawAxis(XInput.XINPUT_WIN_LEFT_Y_AXIS);
			operatorRightJoyX = operatorJoy.getRawAxis(XInput.XINPUT_WIN_RIGHT_X_AXIS);
			operatorRightJoyY = -operatorJoy.getRawAxis(XInput.XINPUT_WIN_RIGHT_Y_AXIS);
		}

		// DriveTrain Variables
		tankDriveDesired = tankDriveButton.isDown();
		arcadeDriveDesired = arcadeDriveButton.isDown();

		// Superstructure Variables
		slowDriveTier1Desired = slowDriveTier1Button.isDown();
		slowDriveTier2Desired = slowDriveTier2Button.isDown();
		driveBackDesired = driveBackButton.isDown();
		driveBackOtherDesired = driveBackOtherButton.isDown();
	}

	public void readAllButtons() {
		arcadeDriveButton.readValue();
		tankDriveButton.readValue();
		slowDriveTier1Button.readValue();
		slowDriveTier2Button.readValue();
		driveBackButton.readValue();
		driveBackOtherButton.readValue();
	}

	public double getJoystickValue(Joysticks j, Axes a) {
		switch (j) {
		case kDriverJoy:
			if (a == Axes.kLX) {
				return driverLeftJoyX;
			} else if (a == Axes.kLY) {
				return driverLeftJoyY;
			} else if (a == Axes.kRX) {
				return driverRightJoyX;
			} else if (a == Axes.kRY) {
				return driverRightJoyY;
			}
			break;
		case kOperatorJoy:
			if (a == Axes.kLX) {
				return operatorLeftJoyX;
			} else if (a == Axes.kLY) {
				return operatorLeftJoyY;
			} else if (a == Axes.kRX) {
				return operatorRightJoyX;
			} else if (a == Axes.kRY) {
				return operatorRightJoyY;
			}
			break;
		default:
			return 0.0;
		}
		return 0.0;
	}

	public boolean getTankDriveDesired() {
		return tankDriveDesired;
	}

	// Returns true if arcade drive is desired
	public boolean getArcadeDriveDesired() {
		return arcadeDriveDesired;
	}

	public boolean getSlowDriveTier1Desired() {
		return slowDriveTier1Desired;
	}

	public boolean getSlowDriveTier2Desired() {
		return slowDriveTier2Desired;
	}

	public boolean getDriveBackDesired() {
		return driveBackDesired;
	}

	public boolean getDriveBackOtherDesired() {
		return driveBackOtherDesired;
	}
}
