/**
 * 
 */
package org.usfirst.frc.team3997.robot.feed;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * @author peter
 *
 */
public class PlateDetector {
	/**
	 * 
	 */
	
	
	public static char getSwitchColor() {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		return gameData.charAt(0);
	}
	public static char getScaleColor() {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		return gameData.charAt(1);
	}
	
	
	

}
