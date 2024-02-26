package frc.robot.MyUtils;

public class Gains {

	public final double kP;
	public final double kI;
	public final double kD;
	public final double kFF;
	public final double kIzone;
	public final double kMinOutput;
	public final double kMaxOutput;
	public final double kMinVel;
	public final double kMaxVel;
	public final double kMaxAcc;
	public final double kAllowedErr;
	public final int kSlotID;

	public Gains(

		double _kP, 
		double _kI, 
		double _kD, 
		double _kFF, 
		double _kIzone, 
		double _kMinOutput, 
		double _kMaxOutput,
		double _kMinVel,
		double _kMaxVel,
		double _kMaxAcc,
		double _kAllowedErr,
		int _kSlotID

	) {	

		kP = _kP;
		kI = _kI;
		kD = _kD;
		kFF = _kFF;
		kIzone = _kIzone;
		kMinOutput = _kMinOutput;
		kMaxOutput = _kMaxOutput;
		kMinVel = _kMinVel;
		kMaxVel = _kMaxVel;
		kMaxAcc = _kMaxAcc;
		kAllowedErr = _kAllowedErr;
		kSlotID = _kSlotID;

	}
}		