package org.usfirst.frc.team3997.robot.auto.actions;

public abstract class Action {
	public abstract boolean isFinished();
	public abstract void update();
	public abstract void finish();
	public abstract void start();
	
	protected double goal_time;
	protected double start_time;
	protected double x_drive;
	protected double y_drive;

}
