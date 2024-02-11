package Team4450.Robot.commands.autonomous;

import static Team4450.Robot.Constants.*;

import Team4450.Lib.Util;
import Team4450.Robot.RobotContainer;
import Team4450.Robot.subsystems.DriveBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/**
 * A command that will turn the robot to the specified angle using a motion
 * profiled PID command. Velocity and accelleration are a guess, need to
 * characterize the robot for good numbers.
 */
public class AutoRotateHdgProfiled extends ProfiledPIDCommand {
	private DriveBase driveBase;

	// private static double kP = .005, kI = .01, kD = 0, kToleranceDeg = 1,
	// kToleranceVelds = 10;
	private static double kP = 2.5, kI = .20, kD = 0, kToleranceRad = 1.0, kToleranceVelrs = 1.0, goal;
	private double startTime, targetHeading;
	private int iterations;

	// We work in degrees but the profile works in radians, so we convert.
	private static double kMaxRotationVelrs = Math.toRadians(MAX_ROTATIONAL_VEL);
	private static double kMaxRotationAccelrss = Math.toRadians(MAX_ROTATIONAL_ACCEL);

	/**
	 * Turns to robot to the specified angle using a motion profile.
	 *
	 * @param drive
	 *            The drive subsystem to use.
	 * @param heading
	 *            The heading to turn to 0 to 359 degrees.
	 */
	public AutoRotateHdgProfiled(DriveBase drive, double heading) {
		super(new ProfiledPIDController(kP, kI, kD,
				new TrapezoidProfile.Constraints(kMaxRotationVelrs, kMaxRotationAccelrss)),
				// Closed loop on heading via reference so pid controller can call it on each
				// execute() call.
				RobotContainer.navx::getYawR,
				// Set target yaw by wrapping goal variable as DoubleSupplier. Will set goal in
				// initialize() below.
				() -> goal,
				// Pipe output to turn robot
				(output, setpoint) -> drive.curvatureDrive(0, output, true),
				// Require the drive
				drive);

		Util.checkRange(heading, 0, 359, "heading");

		Util.consoleLog("heading=%.1f  kP=%.3f  kI=%.3f", heading, kP, kI);

		driveBase = drive;
		targetHeading = heading;

		// Set the controller to be continuous (because it is an angle controller)
		// getController().enableContinuousInput(-180, 180);

		// Set the controller tolerance - the velocity tolerance ensures the robot is
		// stationary at the
		// setpoint before it is considered as having reached the reference
		getController().setTolerance(kToleranceRad, kToleranceVelrs);
	}

	@Override
	public void initialize() {
		Util.consoleLog();

		startTime = Util.timeStamp();

		// Try to prevent over rotation.
		driveBase.SetCANTalonBrakeMode(true);

		// Set target heading then get heading yaw in raidans and save in goal variable.

		RobotContainer.navx.setTargetHeading(targetHeading);

		goal = RobotContainer.navx.getHeadingYawR();

		Util.consoleLog("start hdng=%.2f  tgt=%.2f  yaw=%.2f yawR=%.2f", RobotContainer.navx.getHeading(),
				targetHeading, RobotContainer.navx.getHeadingYaw(), RobotContainer.navx.getHeadingYawR());

		// Reset gyro yaw to zero, wait up to 1 sec for navx to return zero yaw.
		RobotContainer.navx.resetYawWait(1, 1000);
	}

	@Override
	public void execute() {
		super.execute();

		Util.consoleLog("goal=%.2fr  sp=%.5fr  m=%.3fr  err=%.3f goal=%.2f", getController().getGoal().position,
				getController().getSetpoint().position, m_measurement.getAsDouble(), getController().getPositionError(),
				goal);

		Util.consoleLog("yaw=%.2f hdngyaw=%.2f hdng=%.2f lpwr=%.2f rpwr=%.2f", RobotContainer.navx.getYaw(),
				RobotContainer.navx.getHeadingYaw(), RobotContainer.navx.getHeading(), driveBase.getLeftPower(),
				-driveBase.getRightPower());

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
				RobotContainer.navx.getHeadingYaw());

		// Wait for robot to stop moving.
		Util.consoleLog("moving=%b", RobotContainer.navx.isRotating());

		// while (RobotContainer.navx.isRotating()) {Timer.delay(.10);}
		// Util.consoleLog("moving=%b", Devices.navx.isRotating());

		Util.consoleLog("2  hdg=%.2f  yaw=%.2f", RobotContainer.navx.getHeading(), RobotContainer.navx.getHeadingYaw());

		Util.consoleLog("iterations=%d  elapsed time=%.3fs", iterations, Util.getElaspedTime(startTime));

		Util.consoleLog("end -----------------------------------------------------");
	}
}
