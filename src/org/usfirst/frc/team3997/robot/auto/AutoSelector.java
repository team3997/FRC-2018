package org.usfirst.frc.team3997.robot.auto;

import java.util.ArrayList;

import org.usfirst.frc.team3997.robot.MasterController;
import org.usfirst.frc.team3997.robot.auto.routines.CenterGearRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.CustomDistanceRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.DoNothingRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.DriveThreeSecRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.LeftGearRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.PassAutoLineRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.RightGearRoutine;
import org.usfirst.frc.team3997.robot.auto.routines.TurnRoutine;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelector {

	SendableChooser<Integer> autoChooser;
	private ArrayList<AutoRoutine> autoRoutines;
	int selectedIndex = 0;
	
	public AutoSelector(MasterController controllers) {
		//REMEMBER ORDER SAME ORDER AS LIST OPTIONS!!!
		registerAutonomous(new DoNothingRoutine());
		registerAutonomous(new DriveThreeSecRoutine(controllers));
		registerAutonomous(new PassAutoLineRoutine(controllers));
		registerAutonomous(new TurnRoutine(controllers));
		registerAutonomous(new CustomDistanceRoutine(controllers));
		registerAutonomous(new LeftGearRoutine(controllers));
		registerAutonomous(new CenterGearRoutine(controllers));
		registerAutonomous(new RightGearRoutine(controllers));
	} 
	
	public void listOptions() {
		autoChooser = new SendableChooser<Integer>();
		
		autoChooser.addDefault("Nothing (Default)", 0);
		autoChooser.addObject("Drive (3s)", 1);
		autoChooser.addObject("Pass Auto Line and back (Drive 100)", 2);
		autoChooser.addObject("Turn 90 degrees", 3);
		autoChooser.addObject("Custom Routine (check preferences)", 4);
		autoChooser.addObject("Left Gear", 5);
		autoChooser.addObject("Center Gear", 6);
		autoChooser.addObject("Right Gear", 7);

		SmartDashboard.putData("Autonomous: ", autoChooser);
	}
	
	public AutoRoutine pick() {
		setAutoRoutineByIndex((int)autoChooser.getSelected());
		return getAutoRoutine();
	}
	
	public void registerAutonomous(AutoRoutine auto) {
		autoRoutines.add(auto);
	}
	
	public AutoRoutine getAutoRoutine() {
		return autoRoutines.get(selectedIndex);
	}
	
	private void setAutoRoutineByIndex(int input) {
		if(input < 0 || input >= autoRoutines.size()) {
			input = 0;
		}
		selectedIndex = input;
	}
	public AutoRoutine getDefaultRoutine() {
		return autoRoutines.get(0);
	}

}
