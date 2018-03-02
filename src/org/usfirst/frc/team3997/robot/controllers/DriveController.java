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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/** TODO make this comment better: Handles both teleoperated and autonomus driving 
 *  
 * @category controllers
 * @author Peter I. Chacko, Eric Warner, John Sullivan Jr, Jake Boothby, Elliot
 *         Friedlander
 * 
 * **/
public class DriveController {

	private RobotModel robot;
	//Handles the math for arcade, curvature, and tank drive
	private DifferentialDrive drive;
	private RemoteControl humanControl;
	/** Current drive state**/
	private DriveState m_stateVal;
	/** Next drive state**/
	private DriveState nextState;
	/** Left PID Output 
	 * <i>Uses {@link WheelsPIDOutput}</i>**/
	public PIDOutput leftPIDOutput;
	
	public PIDController leftPID;
	/** Left Right PID Output 
	 * <i>Uses {@link WheelsPIDOutput}</i>**/
	public PIDOutput rightPIDOutput;
	public PIDController rightPID;

	// TODO public PIDOutput driveXPIDOutput;
	// TODO public PIDController visionPID;
	// TODO public VisionPIDSource visionPIDSource

	/** Staight PID Output 
	 * <i>Uses {@link ArcadeStraightPIDOutput}</i>**/
	public PIDOutput straightPIDOutput;
	public PIDController straightPID;
	/** Averages Encoder values PID Source 
	 * <i>Uses {@link DriveEncodersPIDSource}</i>**/
	public PIDSource avgEncodersPIDSource;

	/** Different types of drive state
	 * @param kInitialize Initial DriveState
	 * @param kTeleopDrive Teleop DriveState
	 * **/
	enum DriveState {
		kInitialize, kTeleopDrive
	};

	/** Initalizes all drive variables
	 * @param robot Robot Model to get encoders and motors
	 * @param humanControl Get inputs from controllers
	 * **/
	public DriveController(RobotModel robot, RemoteControl humanControl) {
		this.robot = robot;
		this.humanControl = humanControl;

		drive = new DifferentialDrive(this.robot.leftDriveMotors, this.robot.rightDriveMotors);
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
	/** Updates inputs and drive in teleopPeriodic
	 * @param currTimeSec Does absolutely nothing
	 * @param deltaTimeSec Does absolutely nothing **/
	public void update(double currTimeSec, double deltaTimeSec) {
		switch (m_stateVal) {
		case kInitialize:
			leftPID.disable();
			rightPID.disable();
			nextState = DriveState.kTeleopDrive;
			break;
		case kTeleopDrive:
			double driverLeftX = humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kLX);
			double driverLeftY = humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kLY);
			double driverRightX = humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kRX);
			double driverRightY = humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy, RemoteControl.Axes.kRY);

			if (leftPID.isEnabled() || rightPID.isEnabled()) {
				leftPID.disable();
				rightPID.disable();
			}

			if (Params.USE_ARCADE_DRIVE) {
				arcadeDrive(driverLeftY, driverRightX, true);
			} else {
				tankDrive(driverLeftY, driverRightY);
			}

			nextState = DriveState.kTeleopDrive;
			break;
		}
		m_stateVal = nextState;
	}
	/** Arcade Drive function, adds brake functionality
	 * @param myY moveValue
	 * @param myX rotateValue
	 * @param teleOp boolean inTeleop? **/
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

			drive.arcadeDrive(myY * Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER * Params.MAX_SPEED,
					myX * Params.GLOBAL_X_DRIVE_SPEED_MULTIPLIER * Params.MAX_SPEED,
					Params.SQUARE_DRIVE_AXIS_INPUT);
			SmartDashboard.putNumber("Prefs MAXSPEED", Params.MAX_SPEED);

		} else {
			drive.arcadeDrive(myY, myX, false);
		}
	}
	/** Tank Drive Function with multipliers **/
	public void tankDrive(double myLeft, double myRight) {
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
		drive.tankDrive(myLeft * Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER,
				myRight * Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER, Params.SQUARE_DRIVE_AXIS_INPUT);
	}
	/** Changes current DriveState to initalize **/
	public void reset() {
		m_stateVal = DriveState.kInitialize;
	}
	/** Stops driveTrain **/
	public void stop() {
		drive.arcadeDrive(0, 0, false);
	}
}
