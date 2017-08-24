package org.usfirst.frc.team3997.robot.controllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionController {
	private boolean is_enabled;
	private int left_contour, right_contour;
	public VisionController() {
		is_enabled = false;
	}

	public void reset() {
		
	}
	
	public void update() {
		SmartDashboard.putBoolean("VISION_isProcessing", is_enabled);
		//left_contour = SmartDashboard.getNumber("VISION_leftContour", 0.0);
		//right_contour = SmartDashboard.getNumber("VISION_rightContour", 0.0);
	}
	
	public void enable() {
		is_enabled = true;
	}
	
	public void disable() {
		is_enabled = false;
	}
	
	public boolean isEnabled() {
		return is_enabled;
	}
	
	public double getLeftContour() {
		return left_contour;
	}
	
	public double getRightContour() {
		return right_contour;
	}
	
	
}
