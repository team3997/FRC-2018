package org.usfirst.frc.team3997.robot;

import org.usfirst.frc.team3997.robot.auto.AutoRoutineRunner;
import org.usfirst.frc.team3997.robot.controllers.DriveController;
import org.usfirst.frc.team3997.robot.controllers.VisionController;
import org.usfirst.frc.team3997.robot.hardware.ControlBoard;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotModel robot = new RobotModel();
	RemoteControl humanControl = new ControlBoard();
	DriveController driveController = new DriveController(robot, humanControl);
	VisionController visionController = new VisionController();
	
	MasterController masterController = new MasterController(driveController, robot, visionController);
	
	Timer timer = new Timer();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		if(humanControl.getArcadeDriveDesired()) {
			Params.USE_ARCADE_DRIVE = true;
		} else if(humanControl.getTankDriveDesired()) {
			Params.USE_ARCADE_DRIVE = false;
		} 
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		AutoRoutineRunner.getTimer().reset();

		timer.reset();
		timer.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
		robot.resetTimer();
		robot.resetEncoders();
		
		driveController.reset();
		
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		humanControl.readControls();
		driveController.update(0, 0);
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	
	public void disabledInit() {
		AutoRoutineRunner.getTimer().reset();

		if(humanControl.getArcadeDriveDesired()) {
			Params.USE_ARCADE_DRIVE = true;
		} else if(humanControl.getTankDriveDesired()) {
			Params.USE_ARCADE_DRIVE = false;
		} 
	}

	public void disabledPeriodic() {
		AutoRoutineRunner.getTimer().reset();

		if(humanControl.getArcadeDriveDesired()) {
			Params.USE_ARCADE_DRIVE = true;
		} else if(humanControl.getTankDriveDesired()) {
			Params.USE_ARCADE_DRIVE = false;
		} 
	}
}
