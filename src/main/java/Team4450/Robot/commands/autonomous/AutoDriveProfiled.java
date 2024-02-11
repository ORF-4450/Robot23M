package Team4450.Robot.commands.autonomous;

import static Team4450.Robot.Constants.*;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Robot.RobotContainer;
import Team4450.Robot.commands.autonomous.AutoDrive.Brakes;
import Team4450.Robot.commands.autonomous.AutoDrive.StopMotors;
import Team4450.Robot.subsystems.DriveBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/**
 * A command that will drive the robot to the specified distance using a motion
 * profiled PID command and steering correction. Velocity & acceleration are a
 * guess, need to characterize the robot for good numbers.
 */
public class AutoDriveProfiled extends ProfiledPIDCommand {
	private DriveBase driveBase;

	private static double kP = 1.2, kI = .15, kD = 0, kToleranceMeters = .1, curve;
	private double distance, kSteeringGain = .07, startTime;
	private int iterations;
	private StopMotors stop;
	private Brakes brakes;

	/**
	 * Drives robot to the specified distance using a motion profile with straight
	 * steering.
	 *
	 * @param drive
	 *            The drive subsystem to use.
	 * @param distance
	 *            The distance to drive in meters. + is forward.
	 * @param stop
	 *            Set to stop or not stop motors at end.
	 * @param brakes
	 *            If stopping, set brakes on or off.
	 */
	public AutoDriveProfiled(DriveBase drive, double distance, StopMotors stop, Brakes brakes) {
		super(new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(MAX_WHEEL_SPEED, MAX_WHEEL_ACCEL)),
				// Closed loop on encoder distance via reference so pid controller can call it
				// on each execute() call.
				drive::getAvgEncoderDist,
				// Set target distance.
				distance,
				// Pipe output to drive robot
				(output, setpoint) -> drive.arcadeDrive(output, curve, false),
				// Require the drive
				drive);

		Util.consoleLog("distance=%.3fm  stop=%s  brakes=%s", distance, stop, brakes);

		Util.consoleLog("kP=%.6f  kI=%.6f", kP, kI);

		driveBase = drive;
		this.brakes = brakes;
		this.stop = stop;
		this.distance = distance;

		// Set the controller tolerance.
		getController().setTolerance(kToleranceMeters);
	}

	@Override
	public void initialize() {
		Util.consoleLog("distance=%.3fm  stop=%s  brakes=%s", distance, stop, brakes);

		startTime = Util.timeStamp();

		if (brakes == Brakes.on)
			driveBase.SetCANTalonBrakeMode(true);
		else
			driveBase.SetCANTalonBrakeMode(false);

		driveBase.resetEncodersWithDelay();

		RobotContainer.navx.resetYawWait(); // (1, 1000);
	}

	public void execute() {
		Util.consoleLog();

		double yaw = RobotContainer.navx.getYaw();

		curve = Util.clampValue(-yaw * kSteeringGain, 1.0);

		super.execute();

		LCD.printLine(LCD_4, "Auto wheel encoder avg=%d  dist=%.3f", driveBase.getAvgEncoder(),
				driveBase.getAvgEncoderDist());

		Util.consoleLog("tg=%.3f  avdist=%.3f  sp=%.3f err=%.3f  yaw=%.2f curve=%.2f",
				getController().getGoal().position, driveBase.getAvgEncoderDist(),
				getController().getSetpoint().position, getController().getPositionError(), yaw, curve);

		if (RobotBase.isReal())
			Util.consoleLog("lpwr=%.2f  rpwr=%.2f", driveBase.getLeftPower(), driveBase.getRightPower());

		iterations++;
	}

	@Override
	public boolean isFinished() {
		// End when the controller is at the goal.
		return getController().atGoal();
	}

	@Override
	public void end(boolean interrupted) {
		Util.consoleLog("interrupted=%b", interrupted);

		if (stop == StopMotors.stop)
			driveBase.stop();

		double actualDist = Math.abs(driveBase.getAvgEncoderDist());

		Util.consoleLog("end: target=%.3f  actual=%.3f  error=%.2f pct  yaw=%.2f hdng=%.2f", Math.abs(distance),
				actualDist, (actualDist - Math.abs(distance)) / Math.abs(distance) * 100.0,
				RobotContainer.navx.getYaw(), RobotContainer.navx.getHeading());

		Util.consoleLog("iterations=%d  elapsed time=%.3fs", iterations, Util.getElaspedTime(startTime));

		Util.consoleLog("end ---------------------------------------------------------------");
	}
}
