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
	//from 0-1
	public double score(double AreaTargetScore, double AreaCurrentScore) {
		double productCoefficient = (1/(1/AreaTargetScore))*100;
		//from 0-100 Could be greater than 100 though that's why we need to convert
		double rawScore = AreaCurrentScore*productCoefficient;
		//from 0-1
		double convertedScore;
		if(rawScore <= 100) {
			convertedScore = rawScore/100;
			return convertedScore;
		} else {
			convertedScore = 100/rawScore;
			return convertedScore;
		}
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
