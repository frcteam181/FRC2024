package frc.robot.MyUtils;

public class PresetValues {

	public final double kWristPos;
    public final double kArmPos;

	public PresetValues(double _kWristPos, double _kArmPos, boolean isDeg) {	

		if(isDeg) {
			kWristPos = Math.toRadians(_kWristPos);
        	kArmPos = Math.toRadians(_kArmPos);
		} else {
			kWristPos = _kWristPos;
        	kArmPos = _kArmPos;
		}

	}
}		