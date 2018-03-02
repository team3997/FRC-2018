package org.usfirst.frc.team3997.robot.feed;

import org.usfirst.frc.team3997.robot.Params;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardInput {

	Preferences preferences;

	public DashboardInput() {
		preferences = Preferences.getInstance();

		DashboardVariables.firstAutoDistance = preferences.getDouble("First Auto Distance", 0);

		DashboardVariables.nextAutoAngle = preferences.getDouble("Next Auto Angle", 0);

		DashboardVariables.lastAutoDistance = preferences.getDouble("Second Auto Distance", 0);

		Params.arm_p = preferences.getDouble("ARM P Value", 0);
		Params.arm_i = preferences.getDouble("ARM I Value", 0);
		Params.arm_d = preferences.getDouble("ARM D Value", 0);
		Params.arm_f = preferences.getDouble("ARM F Value", 0);

		Params.ARM_CLIMB_SETPOINT = preferences.getDouble("CLIMB_ARM_ANGLE", 150);
		Params.ARM_SCALE_SETPOINT = preferences.getDouble("SCALE_ARM_ANGLE", 120);
		Params.ARM_SWITCH_SETPOINT = preferences.getDouble("SWITCH_ARM_ANGLE", 90);
		Params.ARM_FEED_SETPOINT = preferences.getDouble("FEED_ARM_ANGLE", 10);

		DashboardVariables.max_speed = preferences.getDouble("Max Speed", 1);
		Params.drive_p = preferences.getDouble("Drive P Value", 0);
		Params.drive_i = preferences.getDouble("Drive I Value", 0);
		Params.drive_d = preferences.getDouble("Drive D Value", 0);

		Params.MAX_SPEED = preferences.getDouble("Max Speed", 1);
	}

	public void updateInput() {
		preferences = Preferences.getInstance();

		DashboardVariables.firstAutoDistance = preferences.getDouble("First Auto Distance", 0);

		DashboardVariables.nextAutoAngle = preferences.getDouble("Next Auto Angle", 0);

		DashboardVariables.lastAutoDistance = preferences.getDouble("Second Auto Distance", 0);

		Params.arm_p = preferences.getDouble("ARM P Value", 0);
		Params.arm_i = preferences.getDouble("ARM I Value", 0);
		Params.arm_d = preferences.getDouble("ARM D Value", 0);
		Params.arm_f = preferences.getDouble("ARM F Value", 0);

		Params.ARM_CLIMB_SETPOINT = preferences.getDouble("CLIMB_ARM_ANGLE", 150);
		Params.ARM_SCALE_SETPOINT = preferences.getDouble("SCALE_ARM_ANGLE", 120);
		Params.ARM_SWITCH_SETPOINT = preferences.getDouble("SWITCH_ARM_ANGLE", 90);
		Params.ARM_FEED_SETPOINT = preferences.getDouble("FEED_ARM_ANGLE", 10);

		DashboardVariables.max_speed = preferences.getDouble("Max Speed", 1);
		Params.drive_p = preferences.getDouble("Drive P Value", 0);
		Params.drive_i = preferences.getDouble("Drive I Value", 0);
		Params.drive_d = preferences.getDouble("Drive D Value", 0);

		Params.MAX_SPEED = preferences.getDouble("Max Speed", 1);
	}

}
