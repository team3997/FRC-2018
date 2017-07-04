package org.usfirst.frc.team3997.robot.controllers;

import org.usfirst.frc.team3997.robot.Params;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;
import org.usfirst.frc.team3997.robot.pid.ArcadeStraightPIDOutput;
import org.usfirst.frc.team3997.robot.pid.DriveEncodersPIDSource;
import org.usfirst.frc.team3997.robot.pid.WheelsPIDOutput;
import org.usfirst.frc.team3997.robot.hardware.*;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;

public class DriveController {

	private RobotModel robot;

	private RobotDrive drive;
	private RemoteControl humanControl;
	private DriveState m_stateVal;
	private DriveState nextState;

	public PIDOutput leftPIDOutput;
	public PIDController leftPID;

	public PIDOutput rightPIDOutput;
	public PIDController rightPID;

	// TODO public PIDOutput driveXPIDOutput;
	// TODO public PIDController visionPID;
	// TODO public VisionPIDSource visionPIDSource

	public PIDOutput straightPIDOutput;
	public PIDController straightPID;

	public PIDSource avgEncodersPIDSource;

	enum DriveState {
		kInitialize, kTeleopDrive
	};

	public DriveController(RobotModel robot, RemoteControl humanControl) {
		this.robot = robot;
		this.humanControl = humanControl;

		drive = new RobotDrive(this.robot.leftDriveMotorA, this.robot.leftDriveMotorB, this.robot.rightDriveMotorA,
				this.robot.rightDriveMotorB);
		drive.setSafetyEnabled(false);

		this.robot.leftDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		this.robot.leftDriveEncoder.setSamplesToAverage(Params.DRIVE_Y_PID_SAMPLES_AVERAGE);

		this.robot.rightDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		this.robot.rightDriveEncoder.setSamplesToAverage(Params.DRIVE_Y_PID_SAMPLES_AVERAGE);
		// TODO I think that this is wrong
		leftPIDOutput = new WheelsPIDOutput(RobotModel.Wheels.LeftWheels, this.robot);

		leftPID = new PIDController(0, 0, 0, this.robot.leftDriveEncoder, leftPIDOutput);
		// TODO Might change this to max power variable
		leftPID.setOutputRange(-1.0, 1.0);
		leftPID.setAbsoluteTolerance(0.25);
		leftPID.disable();

		// TODO I think that this is wrong
		rightPIDOutput = new WheelsPIDOutput(RobotModel.Wheels.RightWheels, this.robot);

		rightPID = new PIDController(0, 0, 0, robot.rightDriveEncoder, rightPIDOutput);
		// TODO Might change this to max power variable
		rightPID.setOutputRange(-1.0, 1.0);
		rightPID.setAbsoluteTolerance(0.25);
		rightPID.disable();

		avgEncodersPIDSource = new DriveEncodersPIDSource(this.robot);

		straightPIDOutput = new ArcadeStraightPIDOutput(drive, this.robot);
		straightPID = new PIDController(0, 0, 0, avgEncodersPIDSource, straightPIDOutput);
		// TODO might change this to max power variable
		straightPID.setOutputRange(-1.0, 1.0);
		straightPID.setAbsoluteTolerance(1);
		straightPID.disable();
		// TODO ???? enum kInitialize
		m_stateVal = DriveState.kInitialize;
		nextState = DriveState.kInitialize;

	}

	public void update(double currTimeSec, double deltaTimeSec) {
		switch (m_stateVal) {
		case kInitialize:
			leftPID.disable();
			rightPID.disable();
			nextState = DriveState.kTeleopDrive;
			break;
		case kTeleopDrive:
			double driverLeftX;
			driverLeftX = humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kLX);
			double driverLeftY;
			driverLeftY = humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kLY);
			double driverRightX;
			driverRightX = humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kRX);
			double driverRightY;
			driverRightY = humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kRY);

			if (leftPID.isEnabled() || rightPID.isEnabled()) {
				leftPID.disable();
				rightPID.disable();
			}

			if (Params.USE_ARCADE_DRIVE) {
				arcadeDrive(driverLeftY, -driverRightX, true);
			} else {
				tankDrive(driverLeftY, driverRightY);
			}

			nextState = DriveState.kTeleopDrive;
			break;
		}
		m_stateVal = nextState;
	}

	public void arcadeDrive(double myY, double myX, boolean teleOp) {
		if (teleOp) {
			if ((humanControl.getSlowDriveTier1Desired() && !humanControl.getSlowDriveTier2Desired())
					|| (!humanControl.getSlowDriveTier1Desired() && humanControl.getSlowDriveTier2Desired())) {
				Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER = 0.65;
				Params.GLOBAL_X_DRIVE_SPEED_MULTIPLIER = 0.65;
				Params.SQUARE_DRIVE_AXIS_INPUT = false;
			} else {
				Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER = 1.0;
				Params.GLOBAL_X_DRIVE_SPEED_MULTIPLIER = 1.0;
				Params.SQUARE_DRIVE_AXIS_INPUT = true;
			}

			drive.arcadeDrive(myY * Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER * Params.HARDSET_DRIVE_SPEED_MAX,
					myX * Params.GLOBAL_X_DRIVE_SPEED_MULTIPLIER * Params.HARDSET_DRIVE_SPEED_MAX,
					Params.SQUARE_DRIVE_AXIS_INPUT);
		} else {
			drive.arcadeDrive(myY, myX, false);
		}
	}

	public void tankDrive(double myLeft, double myRight) {
		drive.tankDrive(myLeft * Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER,
				myRight * Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER, Params.SQUARE_DRIVE_AXIS_INPUT);
	}

	public void reset() {
		m_stateVal = DriveState.kInitialize;
	}

	public void stop() {
		drive.arcadeDrive(0, 0, false);
	}
}
