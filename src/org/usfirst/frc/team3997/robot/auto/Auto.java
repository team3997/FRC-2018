package org.usfirst.frc.team3997.robot.auto;

import org.usfirst.frc.team3997.robot.MasterController;

public class Auto {
	public AutoRoutine autoRoutine;
	public AutoSelector selector;
	public AutoRoutineRunner runner;
	
	
	public Auto(MasterController controllers) {
		selector = new AutoSelector(controllers);
		runner = new AutoRoutineRunner();
		autoRoutine = selector.getDefaultRoutine();
		
	}
	
	
	public void reset() {
		AutoRoutineRunner.getTimer().reset();
		
	}
	
	public void listOptions() {
		selector.listOptions();
	}
	
	public void start() {
		autoRoutine = selector.pick();
		runner.setAutoRoutine(autoRoutine);
		
		autoRoutine.prestart();
		
		runner.start();
	}
	
	public void stop() {
		AutoRoutineRunner.getTimer().reset();
		AutoRoutineRunner.getTimer().stop();
		runner.stop();
		autoRoutine.m_active = false;

	}

}
