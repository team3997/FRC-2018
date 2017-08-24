package org.usfirst.frc.team3997.robot.pid;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;

public class AnalogPIDSource implements PIDSource {

	Potentiometer analogInput;

	public AnalogPIDSource(Potentiometer analogInput) {
		this.analogInput = analogInput;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		analogInput.setPIDSourceType(pidSource);
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return analogInput.getPIDSourceType();
	}

	@Override
	public double pidGet() {
		switch (getPIDSourceType()) {
		case kDisplacement:
			return analogInput.get();
		case kRate:
			return 0.0;
		default:
			return 0;
		}
	}

}
