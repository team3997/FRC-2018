/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.actions;

import org.usfirst.frc.team3997.robot.controllers.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team3997.robot.MasterController;

/**
 * @author peter
 *
 */


public class DriveIntervalAction extends Action {
	private DriveController kDrive;
	
	public DriveIntervalAction(MasterController controllers, double seconds, double y, double x) {
		goal_time = seconds;
		x_drive = x;
		y_drive = y;
		this.kDrive = controllers.getDriveController();
	}
	
	public boolean isFinished() {
		return (Timer.getFPGATimestamp() >= start_time + goal_time);
	}

	@Override
	public void update() {
		SmartDashboard.putNumber("reachedUPDATE", Timer.getFPGATimestamp());
		kDrive.arcadeDrive(y_drive, x_drive, false);
	}

	@Override
	public void finish() {
		kDrive.stop();
	}

	@Override
	public void start() {
		SmartDashboard.putNumber("reachedSTART", Timer.getFPGATimestamp());
		start_time = Timer.getFPGATimestamp();
	}
}
