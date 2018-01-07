/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.controllers.MotionController;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;

import jaci.pathfinder.Trajectory;

/**
 * @author peter
 *
 */
public class CenterAutoRoutine extends AutoRoutine{
	private MasterController controllers;
	Trajectory trajectory;
	public CenterAutoRoutine(MasterController controllers) {
		this.controllers = controllers;
	}

	@Override
	public void prestart() {
		boolean isLeftSwitch = (PlateDetector.getPlateColor() == 'L');
		if(isLeftSwitch) {
			//trajectory = MotionController.generateTrajectory(centerLeftPath);
		} else {
			// trajectory = MotionController.generateTrajectory(centerRightPath);
		}
	}

	@Override
	protected void routine() {
		//pathFollower(controllers, trajectory, timeout);
	}


}
