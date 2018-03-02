/**
 * 
 */
package org.usfirst.frc.team3997.robot.auto.routines;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.AutoRoutine;
import org.usfirst.frc.team3997.robot.feed.PlateDetector;

import jaci.pathfinder.Trajectory;

/**
 * @author peter
 *
 */
public class LeftAutoRoutine extends AutoRoutine{
	private MasterController controllers;
	Trajectory trajectory;
	
	public LeftAutoRoutine(MasterController controllers) {
		this.controllers = controllers;
	}

	@Override
	public void prestart() {
		boolean isLeftSwitch = (PlateDetector.getSwitchColor() == 'L');
		if(isLeftSwitch) {
			boolean isLeftScale = (PlateDetector.getScaleColor() == 'L');
			if(isLeftScale) {
				
			} else {
				
			}
			
			// trajectory = MotionController.generateTrajectory(rightLeftPath);
		} else {
			boolean isLeftScale = (PlateDetector.getScaleColor() == 'L');
			if(isLeftScale) {
				
			} else {
				
			}
			// trajectory = MotionController.generateTrajectory(rightRightPath);
		}
	}

	@Override
	protected void routine() {
		//pathFollower(controllers, trajectory, timeout);
	}


}
