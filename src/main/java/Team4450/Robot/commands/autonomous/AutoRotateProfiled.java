package Team4450.Robot.commands.autonomous;

import static Team4450.Robot.Constants.*;

import Team4450.Lib.Util;
import Team4450.Robot.RobotContainer;
import Team4450.Robot.subsystems.DriveBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/**
 * A command that will turn the robot to the specified angle using a motion
 * profiled PID command. Velocity and accelleration are a guess, need to
 * characterize the robot for good numbers. With characterized gains, we can try
 * the feed forward...it does not work well with the esitmated/guessed gains.
 */
public class AutoRotateProfiled extends ProfiledPIDCommand {
	private DriveBase driveBase;

	private static AutoRotateProfiled thisInstance;

	// Note: kP was 15 but sim does not work right as 15 causes to much rotation. 5
	// works in sim.
	// But, 5 may be too little for a real robot and not output enough power to
	// move. Need to test.
	private static double kP = 5, kI = kP / 100, kD = 0, kToleranceRad = 1.0, kToleranceVelrs = 1.0;
	private double targetAngle, startTime;
	private int iterations;

	// We work in degrees but the profile works in radians, so we convert. 70 d/s is
	// an eyeball
	// estimate of rotational vel and acceleration is a guess.
	private static double kMaxRotationVelrs = Math.toRadians(MAX_ROTATIONAL_VEL);
	private static double kMaxRotationAccelrss = Math.toRadians(MAX_ROTATIONAL_ACCEL);

	// Estimate both feed forward gains. Feed forward does not seem to work
	// but can't say for sure until we get the bot characterized and get the
	// measured gains.
	private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(DB_KS, DB_KV);

	/**
	 * Turns to robot to the specified angle using a motion profile.
	 *
	 * @param drive
	 *            The drive subsystem to use.
	 * @param targetAngle
	 *            The angle to turn to -180 to +180 degrees, + is clockwise.
	 */
	public AutoRotateProfiled(DriveBase drive, double targetAngle) {
		// Since we are extending ProfiledPIDCommand, we will call the underlying
		// constructor
		// to instantiate the components of ProfiledPIDCommand. For reasons I cannot
		// explain,
		// the PID function won't work with degrees but will work with radians.
		super(new ProfiledPIDController(kP, kI, kD,
				new TrapezoidProfile.Constraints(kMaxRotationVelrs, kMaxRotationAccelrss)),
				// Closed loop on yaw via reference so pid controller can call it on each
				// execute() call.
				RobotContainer.navx::getYawR,
				// Set target angle
				Math.toRadians(targetAngle),
				// Pipe output to turn robot
				// (output, setpoint) -> thisInstance.driveWithFeedForward(output, setpoint),
				(output, setpoint) -> drive.curvatureDrive(0, output, true),
				// Require the drive base
				drive);

		Util.checkRange(targetAngle, 180, "target angle");

		Util.consoleLog("angle=%.2f  kP=%.3f  kI=%.3f", targetAngle, kP, kI);

		driveBase = drive;
		this.targetAngle = targetAngle;
		thisInstance = this;

		// Set the controller tolerance - the velocity tolerance ensures the robot is
		// stationary at the
		// setpoint before it is considered as having reached the reference
		getController().setTolerance(kToleranceRad, kToleranceVelrs);
	}

	// Drive combining feed forward with PID output (angle error). The set point
	// computed
	// by the motion profile feeds the feed forward calculation. So the profile
	// drives both
	// the feed forward (base power setting in volts) and we add in a factor from
	// the PID
	// based on the error between angle setpoint and measured angle.
	// Update: this code does not work at all. Not sure why but will wait to
	// experiment with
	// this after characterization. It may be that the kP value used with FF is
	// quite a bit
	// different than the 2.0 used without FF. However, the FF voltage is WAY off
	// and that
	// may be the result of guessing the FF gains.
	// private void driveWithFeedForward(double power, TrapezoidProfile.State
	// setPoint)
	// {
	// double ff = feedForward.calculate(setPoint.position); //, setPoint.velocity);

	// Util.consoleLog("pwr=%.3f pwr*12=%.3f spp=%.3f spv=%.3f ff=%.3fv", power,
	// power * 12, setPoint.position, setPoint.velocity, ff);

	// // Feed forward is in volts so we convert the PID output (in radians) to
	// // voltage so it combines with ff and then we set motors with voltage.
	// power = power + ff;

	// driveBase.curvatureDrive(0, power, true);
	// }

	@Override
	public void initialize() {
		Util.consoleLog();

		startTime = Util.timeStamp();

		// Try to prevent over rotation.
		driveBase.SetCANTalonBrakeMode(true);

		// Reset gyro yaw to zero, wait up to 1 sec for navx to return zero yaw.
		RobotContainer.navx.resetYawWait(); // 1, 1000);
	}

	@Override
	public void execute() {
		// Run the underlying ProfiledPIDCommand which calls ProfiledPIDController to
		// compute
		// the motor outputs and send them to the consumer we defined above
		// (curvatureDrive).
		super.execute();

		Util.consoleLog("goal=%.2fr  sp=%.3fr  m=%.3fr  err=%.3f", getController().getGoal().position,
				getController().getSetpoint().position, m_measurement.getAsDouble(),
				getController().getPositionError());

		Util.consoleLog("yaw=%.2f  hdng=%.2f  lpwr=%.2f  rpwr=%.2f", RobotContainer.navx.getYaw(),
				RobotContainer.navx.getHeading(), driveBase.getLeftPower(), -driveBase.getRightPower());

		iterations++;
	}

	@Override
	public boolean isFinished() {
		// End when the controller is at the reference.
		return getController().atGoal();
	}

	@Override
	public void end(boolean interrupted) {
		Util.consoleLog("interrupted=%b", interrupted);

		driveBase.stop();

		Util.consoleLog("after stop  hdg=%.2f  yaw=%.2f", RobotContainer.navx.getHeading(),
				RobotContainer.navx.getYaw());

		// Wait for robot to stop moving.
		Util.consoleLog("moving=%b", RobotContainer.navx.isRotating());

		// while (RobotContainer.navx.isRotating()) {Timer.delay(.10);}
		// Util.consoleLog("moving=%b", Devices.navx.isRotating());

		Util.consoleLog("2  hdg=%.2f  yaw=%.2f", RobotContainer.navx.getHeading(), RobotContainer.navx.getYaw());

		Util.consoleLog("iterations=%d  elapsed time=%.3fs", iterations, Util.getElaspedTime(startTime));

		Util.consoleLog("end ----------------------------------------------------------");
	}
}
