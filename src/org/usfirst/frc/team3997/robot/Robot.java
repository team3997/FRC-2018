package org.usfirst.frc.team3997.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team3997.robot.auto.Auto;
import org.usfirst.frc.team3997.robot.auto.AutoRoutineRunner;
import org.usfirst.frc.team3997.robot.controllers.ClimberController;
import org.usfirst.frc.team3997.robot.controllers.DriveController;
import org.usfirst.frc.team3997.robot.controllers.GearController;
import org.usfirst.frc.team3997.robot.controllers.LightController;
import org.usfirst.frc.team3997.robot.controllers.MotionController;
import org.usfirst.frc.team3997.robot.controllers.VisionController;
import org.usfirst.frc.team3997.robot.feed.DashboardInput;
import org.usfirst.frc.team3997.robot.feed.DashboardLogger;
import org.usfirst.frc.team3997.robot.hardware.ControlBoard;
import org.usfirst.frc.team3997.robot.hardware.RemoteControl;
import org.usfirst.frc.team3997.robot.hardware.RobotModel;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
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
	double currTimeSec = 0;
	double lastTimeSec = 0;
	double deltaTimeSec = 0;
	
	RobotModel robot = new RobotModel();
	RemoteControl humanControl = new ControlBoard();
	DriveController driveController = new DriveController(robot, humanControl);
	VisionController visionController = new VisionController();
	LightController lights = new LightController();
	DashboardLogger dashboardLogger = new DashboardLogger(robot, humanControl);
	DashboardInput input = new DashboardInput();
	MotionController motion = new MotionController(robot);
	GearController gearController = new GearController(robot, humanControl);
	ClimberController climberController = new ClimberController(robot, humanControl);
	
	MasterController masterController = new MasterController(driveController, robot, gearController, motion, visionController, lights);

	Auto auto = new Auto(masterController);
	Timer timer = new Timer();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
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
		new Thread(() -> {
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
        }).start();

	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		AutoRoutineRunner.getTimer().reset();
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
		gearController.reset();
		driveController.reset();
		visionController.enable();
		climberController.reset();
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		dashboardLogger.updateData();
		lastTimeSec = currTimeSec;
		currTimeSec = robot.getTime();
		deltaTimeSec = currTimeSec - lastTimeSec;
		humanControl.readControls();
		driveController.update(currTimeSec, deltaTimeSec);
		visionController.disable();
		gearController.update();
		lights.setEnabledLights();
		climberController.update();

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
		//Drive Train
		LiveWindow.addActuator("Drive Train", "Front Left Motor", robot.leftDriveMotorA);
		LiveWindow.addActuator("Drive Train", "Back Left Motor", robot.leftDriveMotorB);
		LiveWindow.addActuator("Drive Train", "Front Right Motor", robot.rightDriveMotorA);
		LiveWindow.addActuator("Drive Train", "Back Right Motor", robot.rightDriveMotorB);
		LiveWindow.addSensor("Drive Train", "Left Encoder", robot.leftDriveEncoder);
		LiveWindow.addSensor("Drive Train", "Right Encoder", robot.rightDriveEncoder);

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
		input.updateInput();

	}

	public void disabledPeriodic() {
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
