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
	
	
	public static char getPlateColor() {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		return gameData.charAt(0);
	}
	
	
	

}
