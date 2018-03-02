package org.usfirst.frc.team3997.robot.pid;

import org.usfirst.frc.team3997.robot.hardware.AbsoluteEncoder;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class AbsoluteEncoderPIDSource implements PIDSource {
	AbsoluteEncoder absoluteEncoder;

	public AbsoluteEncoderPIDSource(AbsoluteEncoder absoluteEncoder) {
		this.absoluteEncoder = absoluteEncoder;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		absoluteEncoder.setPIDSourceType(pidSource);
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return absoluteEncoder.getPIDSourceType();
	}

	@Override
	public double pidGet() {
		switch (getPIDSourceType()) {
		case kDisplacement:
			return absoluteEncoder.getAngle();
		case kRate:
			return 0.0;
		default:
			return 0;
		}
	}

}