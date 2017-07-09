package org.usfirst.frc.team3997.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

public class WaitAction extends Action {
	
	public WaitAction(double seconds) {
		goal_time = seconds;
		
	}
	
	public boolean isFinished() {
		return (Timer.getFPGATimestamp() >= start_time + goal_time);
	}

	@Override
	public void update() {
		
	}

	@Override
	public void finish() {
	}

	@Override
	public void start() {
		start_time = Timer.getFPGATimestamp();
	}
}
