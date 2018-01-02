package org.usfirst.frc.team3997.robot;

import org.usfirst.frc.team3997.robot.auto.Auto;
import org.usfirst.frc.team3997.robot.auto.AutoRoutineRunner;
import org.usfirst.frc.team3997.robot.auto.actions.Action;
import org.usfirst.frc.team3997.robot.auto.actions.DriveIntervalAction;
import org.usfirst.frc.team3997.robot.controllers.DriveController;
import org.usfirst.frc.team3997.robot.controllers.LightController;
import org.usfirst.frc.team3997.robot.controllers.MotionController;
import org.usfirst.frc.team3997.robot.controllers.VisionController;
import org.usfirst.frc.team3997.robot.feed.DashboardInput;
import org.usfirst.frc.team3997.robot.feed.DashboardLogger;
import org.usfirst.frc.team3997.robot.hardware.ControlBoard;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	double currTimeSec = 0;
	double lastTimeSec = 0;
	double deltaTimeSec = 0;
	
	RobotModel robot;
	RemoteControl humanControl;
	DriveController driveController;
	VisionController visionController;
	LightController lights;
	DashboardLogger dashboardLogger;
	DashboardInput input;

	MotionController motion;

	MasterController masterController;
	Auto auto;
	Timer timer;


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		robot = new RobotModel();
		humanControl = new ControlBoard();
		driveController = new DriveController(robot, humanControl);
		visionController = new VisionController();
		lights = new LightController();
		dashboardLogger = new DashboardLogger(robot, humanControl);
		input = new DashboardInput();
		motion = new MotionController(robot);
		
		masterController = new MasterController(driveController, robot, motion, visionController,
				lights);
		auto = new Auto(masterController);
		timer = new Timer();
		
		
		lights.setEnabledLights();
		auto.reset();
		auto.listOptions();

		input.updateInput();

		if (humanControl.getArcadeDriveDesired()) {
			Params.USE_ARCADE_DRIVE = true;
		} else if (humanControl.getTankDriveDesired()) {
			Params.USE_ARCADE_DRIVE = false;
		}
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
		/*new Thread(() -> {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(640, 480);
            
            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Line", 640, 480);
            
            Mat output = new Mat();
            
            while(!Thread.interrupted()) {
                cvSink.grabFrame(output);
                int thickness = 2;
                Imgproc.line(output, new Point((output.size().width / 2), 0), new Point((output.size().width / 2), (output.size().height)), new Scalar(0, 0, 0), thickness);
                outputStream.putFrame(output);
            }
        }).start();*/

	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		robot.reset();
		robot.updateGyro();
		AutoRoutineRunner.getTimer().reset();
		input.updateInput();
		auto.stop();

		timer.reset();
		timer.start();

		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;

		auto.start();
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		robot.updateGyro();
		SmartDashboard.putNumber("gyro", robot.getAngle());
		visionController.update();
		lights.setAutoLights();
		dashboardLogger.updateData();
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
		auto.stop();
		robot.resetTimer();
		robot.resetEncoders();
		driveController.reset();
		visionController.enable();
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putNumber("gyro", robot.getAngle());

		dashboardLogger.updateData();
		lastTimeSec = currTimeSec;
		currTimeSec = robot.getTime();
		deltaTimeSec = currTimeSec - lastTimeSec;
		humanControl.readControls();
		driveController.update(currTimeSec, deltaTimeSec);
		visionController.disable();
		lights.setEnabledLights();

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		input.updateInput();

	}

	public void disabledInit() {
		AutoRoutineRunner.getTimer().reset();
		visionController.disable();
		driveController.reset();
		if (humanControl.getArcadeDriveDesired()) {
			Params.USE_ARCADE_DRIVE = true;
		} else if (humanControl.getTankDriveDesired()) {
			Params.USE_ARCADE_DRIVE = false;
		}
		robot.reset();
		input.updateInput();

	}

	public void disabledPeriodic() {
		robot.reset();

		input.updateInput();
		dashboardLogger.updateData();

		AutoRoutineRunner.getTimer().reset();
		humanControl.readControls();
		visionController.update();
		if (humanControl.getArcadeDriveDesired()) {
			Params.USE_ARCADE_DRIVE = true;
		} else if (humanControl.getTankDriveDesired()) {
			Params.USE_ARCADE_DRIVE = false;
		}

		lights.setDisabledLights();
	}

}
