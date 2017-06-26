package org.usfirst.frc.team3997.robot.hardware;

import edu.wpi.first.wpilibj.*;
import org.usfirst.frc.team3997.robot.hardware.Ports;
public class RobotModel {
	
	
	public Spark leftDriveMotorA, leftDriveMotorB, rightDriveMotorA, rightDriveMotorB;
	public Encoder leftDriveEncoder, rightDriveEncoder;
	public CameraServer camera;
	public Timer timer;
	
	private PowerDistributionPanel pdp;
	private double leftDriveACurrent, leftDriveBCurrent, rightDriveACurrent, rightDriveBCurrent;
	
	public RobotModel() {
		pdp = new PowerDistributionPanel();
		//Init drive motors
		leftDriveMotorA = new Spark(Ports.LEFT_DRIVE_MOTOR_A_PWM_PORT);
		leftDriveMotorB = new Spark(Ports.LEFT_DRIVE_MOTOR_B_PWM_PORT);
		rightDriveMotorA = new Spark(Ports.RIGHT_DRIVE_MOTOR_A_PWM_PORT);
		rightDriveMotorB = new Spark(Ports.RIGHT_DRIVE_MOTOR_B_PWM_PORT);
	
		leftDriveEncoder = new Encoder(Ports.LEFT_DRIVE_ENCODER_PORTS[0], Ports.LEFT_DRIVE_ENCODER_PORTS[1]);
		rightDriveEncoder = new Encoder(Ports.RIGHT_DRIVE_ENCODER_PORTS[0], Ports.RIGHT_DRIVE_ENCODER_PORTS[1]);
	
		leftDriveEncoder.setReverseDirection(false);
		leftDriveEncoder.setDistancePerPulse(((1.0) / (250.0)) * ((4.0) * (Math.PI)));
		leftDriveEncoder.setSamplesToAverage(1);
		rightDriveEncoder.setReverseDirection(false);
		rightDriveEncoder.setDistancePerPulse(((1.0) / (250.0)) * ((4.0) * (Math.PI)));
		rightDriveEncoder.setSamplesToAverage(1);

		leftDriveMotorA.setSafetyEnabled(false);
		leftDriveMotorB.setSafetyEnabled(false);
		rightDriveMotorA.setSafetyEnabled(false);
		rightDriveMotorB.setSafetyEnabled(false);
		
		leftDriveMotorA.setInverted(false);
		leftDriveMotorB.setInverted(false);
		rightDriveMotorA.setInverted(false);
		rightDriveMotorB.setInverted(false);

		leftDriveACurrent = 0;
		leftDriveBCurrent = 0;
		rightDriveACurrent = 0;
		rightDriveBCurrent = 0;
		
		timer = new Timer();
		timer.start();
		//TODO add real url
		camera.addServer("Server");

		
	}
	public enum Wheels {
		LeftWheels, RightWheels, AllWheels
	};

	//sets the speed for a given wheel(s)
	public void setWheelSpeed(Wheels w, double speed) {
		switch (w) {
		case LeftWheels:
			leftDriveMotorA.set(speed);
			leftDriveMotorB.set(speed);
			break;
		case RightWheels:
			rightDriveMotorA.set(-speed); //negative value since wheels are inverted on robot
			rightDriveMotorB.set(-speed); //negative value since wheels are inverted on robot
			break;
		case AllWheels:
			leftDriveMotorA.set(speed);
			leftDriveMotorB.set(speed);
			rightDriveMotorA.set(-speed); //negative value since wheels are inverted on robot
			rightDriveMotorB.set(-speed); //negative value since wheels are inverted on robot
			break;
		}
	}
	
	//returns the speed of a given wheel
	public double getWheelSpeed(Wheels w) {
		switch (w) {
		case LeftWheels:
			return leftDriveMotorA.get();
		case RightWheels:
			return -rightDriveMotorA.get();
		default:
			return 0;
		}	}
	//resets variables and objects
	public void reset() {
		resetEncoders();
	}
	
	//initializes variables pertaining to current
	public void updateCurrent() {
		leftDriveACurrent = pdp.getCurrent(Ports.LEFT_DRIVE_MOTOR_A_PDP_CHAN);
		leftDriveBCurrent = pdp.getCurrent(Ports.LEFT_DRIVE_MOTOR_B_PDP_CHAN);
		rightDriveACurrent = pdp.getCurrent(Ports.RIGHT_DRIVE_MOTOR_A_PDP_CHAN);
		rightDriveBCurrent = pdp.getCurrent(Ports.RIGHT_DRIVE_MOTOR_B_PDP_CHAN);
	}
	
	//returns the voltage
	public double getVoltage() {
		return pdp.getVoltage();
	}
	
	//returns the total energy of the PDP
	public double getTotalEnergy() {
		return pdp.getTotalEnergy();
	}
	
	//returns the total current of the PDP
	public double getTotalCurrent() {
		return pdp.getTotalCurrent();
	}
	
	//returns the total power of the PDP
	public double getTotalPower() {
		return pdp.getTotalPower();
	}
	
	 //returns the current of a given channel
	public double getCurrent(int channel) {
		switch (channel) {
		case Ports.RIGHT_DRIVE_MOTOR_A_PDP_CHAN:
			return rightDriveACurrent;
		case Ports.RIGHT_DRIVE_MOTOR_B_PDP_CHAN:
			return rightDriveBCurrent;
		case Ports.LEFT_DRIVE_MOTOR_A_PDP_CHAN:
			return leftDriveACurrent;
		case Ports.LEFT_DRIVE_MOTOR_B_PDP_CHAN:
			return leftDriveBCurrent;
		default:
			return -1;
		}
	}
	
	//resets the timer
	public void resetTimer() {
		timer.reset();
	}
	
	//returns the time
	public double getTime() {
		return timer.get();
	}
	
	// encoders
	public void resetEncoders() {
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();
	}
	
	public double getEncoderError() {
		return leftDriveEncoder.getDistance() - rightDriveEncoder.getDistance();
	}

	
}
