package Team4450.Robot.commands.autonomous;

import static Team4450.Robot.Constants.*;

import Team4450.Lib.LCD;
import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot.RobotContainer;
import Team4450.Robot.subsystems.DriveBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDrive extends CommandBase {
	private final DriveBase driveBase;

	private double yaw, kSteeringGain = .07, elapsedTime = 0;
	private double kP = .0003, kI = .0003, kD = 0;
	private double power, startTime;
	private int encoderCounts, iterations, targetHeading;
	private boolean driveToHeading;
	private StopMotors stop;
	private Brakes brakes;
	private Pid pid;
	private Heading heading;

	SynchronousPID pidController = null;

	/**
	 * Creates a new AutoDrive command.
	 *
	 * Auto drive straight in set direction and power for specified encoder count.
	 * Stops with or without brakes on CAN bus drive system. Uses NavX yaw to drive
	 * straight.
	 *
	 * @param driveBase
	 *            The DriveBase subsystem used by this command to drive the robot.
	 * @param power
	 *            Power % applied, + is forward.
	 * @param encoderCounts
	 *            Target encoder counts to move, always +.
	 * @param stop
	 *            Stop stops motors at end of move, dontStop leaves power on to flow
	 *            into next move.
	 * @param brakes
	 *            Brakes on or off.
	 * @param pid
	 *            On is use PID to control movement, off is simple drive.
	 * @param heading
	 *            Heading is measure steering yaw from last set navx target heading,
	 *            angle is measure yaw from direction robot is pointing when driving
	 *            starts.
	 *
	 *            Note: This routine is designed for tank drive and the
	 *            P,I,D,steering gain values will likely need adjusting for each new
	 *            drive base as gear ratios and wheel configuration may require
	 *            different values to stop smoothly and accurately.
	 */
	public AutoDrive(DriveBase driveBase, double power, int encoderCounts, StopMotors stop, Brakes brakes, Pid pid,
			Heading heading) {
		this.driveBase = driveBase;

		Util.consoleLog("pwr=%.2f  count=%d  stop=%s  brakes=%s  pid=%s  hdg=%s", power, encoderCounts, stop, brakes,
				pid, heading);

		Util.checkRange(power, 1.0);

		if (encoderCounts < 1)
			throw new IllegalArgumentException("Encoder counts < 1");

		this.power = power;
		this.encoderCounts = encoderCounts;
		this.stop = stop;
		this.brakes = brakes;
		this.pid = pid;
		this.heading = heading;

		// This math computes an appropiate P value based on the magnitude of distance
		// to travel. Again, may
		// or may not work for every drive train configuration.
		// kP = Math.abs(power) / encoderCounts * 2;
		// kI = kP / 10.0 * 2.0;

		Util.consoleLog("kP=%.6f  kI=%.6f", kP, kI);

		addRequirements(this.driveBase);
	}

	/**
	 * Creates a new AutoDrive command.
	 *
	 * Auto drive straight in set direction and power for specified encoder count.
	 * Stops with or without brakes on CAN bus drive system. Uses NavX yaw to drive
	 * straight.
	 *
	 * @param driveBase
	 *            The DriveBase subsystem used by this command to drive the robot.
	 * @param power
	 *            Power % applied, + is forward.
	 * @param distance
	 *            Target distance to move in feet, always +. Enter whole numbers as
	 *            n.0 so Java will recognize the number as a double instead of an
	 *            integer (other constructor).
	 * @param stop
	 *            Stop stops motors at end of move, dontStop leaves power on to flow
	 *            into next move.
	 * @param brakes
	 *            Brakes on or off.
	 * @param pid
	 *            On is use PID to control movement, off is simple drive.
	 * @param heading
	 *            Heading is measure steering yaw from last set navx target heading,
	 *            angle is measure yaw from direction robot is pointing when driving
	 *            starts.
	 *
	 *            Note: This routine is designed for tank drive and the
	 *            P,I,D,steering gain values will likely need adjusting for each new
	 *            drive base as gear ratios and wheel configuration may require
	 *            different values to stop smoothly and accurately.
	 */

	public AutoDrive(DriveBase driveBase, double power, double distance, StopMotors stop, Brakes brakes, Pid pid,
			Heading heading) {
		this(driveBase, power, SRXMagneticEncoderRelative.getTicksForDistance(distance, DRIVE_WHEEL_DIAMETER), stop,
				brakes, pid, heading);
	}

	/**
	 * Creates a new AutoDrive command.
	 *
	 * Auto drive straight in set direction and power for specified encoder count.
	 * Stops with or without brakes on CAN bus drive system. Uses NavX yaw to drive
	 * straight.
	 *
	 * @param driveBase
	 *            The DriveBase subsystem used by this command to drive the robot.
	 * @param power
	 *            Power % applied, + is forward.
	 * @param encoderCounts
	 *            Target encoder counts to move, always +.
	 * @param stop
	 *            Stop stops motors at end of move, dontStop leaves power on to flow
	 *            into next move.
	 * @param brakes
	 *            Brakes on or off.
	 * @param pid
	 *            On is use PID to control movement, off is simple drive.
	 * @param heading
	 *            Target heading 0-359 to drive. Note robot is expected to point
	 *            more or less in this direction at start and this method will try
	 *            to drive to that heading. This method is not expected to rotate or
	 *            turn to this heading from a starting heading signficantly
	 *            different from the target.
	 *
	 *            Note: This routine is designed for tank drive and the
	 *            P,I,D,steering gain values will likely need adjusting for each new
	 *            drive base as gear ratios and wheel configuration may require
	 *            different values to stop smoothly and accurately.
	 */

	public AutoDrive(DriveBase driveBase, double power, int encoderCounts, StopMotors stop, Brakes brakes, Pid pid,
			int targetHeading) {
		this(driveBase, power, encoderCounts, stop, brakes, pid, Heading.heading);

		this.targetHeading = targetHeading;
		driveToHeading = true;
	}

	/**
	 * Creates a new AutoDrive command.
	 *
	 * Auto drive straight in set direction and power for specified encoder count.
	 * Stops with or without brakes on CAN bus drive system. Uses NavX yaw to drive
	 * straight.
	 *
	 * @param driveBase
	 *            The DriveBase subsystem used by this command to drive the robot.
	 * @param power
	 *            Power % applied, + is forward.
	 * @param distance
	 *            Target distance to move in feet, always +. Enter whole numbers as
	 *            n.0 so Java will recognize the number as a double instead of an
	 *            integer (other constructor).
	 * @param stop
	 *            Stop stops motors at end of move, dontStop leaves power on to flow
	 *            into next move.
	 * @param brakes
	 *            Brakes on or off.
	 * @param pid
	 *            On is use PID to control movement, off is simple drive.
	 * @param heading
	 *            Target heading 0-359 to drive. Note robot is expected to point
	 *            more or less in this direction at start and this method will try
	 *            to drive to that heading. This method is not expected to rotate or
	 *            turn to this heading from a starting heading signficantly
	 *            different from the target.
	 *
	 *            Note: This routine is designed for tank drive and the
	 *            P,I,D,steering gain values will likely need adjusting for each new
	 *            drive base as gear ratios and wheel configuration may require
	 *            different values to stop smoothly and accurately.
	 */

	public AutoDrive(DriveBase driveBase, double power, double distance, StopMotors stop, Brakes brakes, Pid pid,
			int targetHeading) {
		this(driveBase, power, SRXMagneticEncoderRelative.getTicksForDistance(distance, DRIVE_WHEEL_DIAMETER), stop,
				brakes, pid, Heading.heading);

		this.targetHeading = targetHeading;
		driveToHeading = true;
	}

	@Override
	public void initialize() {
		Util.consoleLog("pwr=%.2f  count=%d  stop=%s  brakes=%s  pid=%s  hdg=%s  tgthdg=%d", power, encoderCounts, stop,
				brakes, pid, heading, targetHeading);

		startTime = Util.timeStamp();

		if (brakes == Brakes.on)
			driveBase.SetCANTalonBrakeMode(true);
		else
			driveBase.SetCANTalonBrakeMode(false);

		driveBase.resetEncodersWithDelay();

		// If not measuring yaw from current heading, reset yaw based on current
		// direction robot is facing.

		if (heading == Heading.angle) {
			Util.consoleLog("yaw before reset=%.2f  hdg=%.2f", RobotContainer.navx.getYaw(),
					RobotContainer.navx.getHeading());

			RobotContainer.navx.resetYawWait();

			// Note, under simulation this yaw will not show zero until next execution of
			// DriveBase.simulationPeriodic.
			Util.consoleLog("yaw after reset=%.2f  hdg=%.2f", RobotContainer.navx.getYaw(),
					RobotContainer.navx.getHeading());
		} else {
			// When driving to a heading, that heading can be set externally before this
			// command executes
			// or when this command executes (target passed in constructor).
			if (driveToHeading)
				RobotContainer.navx.setTargetHeading(targetHeading);
		}

		// If using PID to control distance, configure the PID object.

		if (pid == Pid.on) {
			pidController = new SynchronousPID(kP, kI, kD);

			if (power < 0) {
				pidController.setSetpoint(-encoderCounts);
				pidController.setOutputRange(power, 0);
			} else {
				pidController.setSetpoint(encoderCounts);
				pidController.setOutputRange(0, power);
			}

			// Start elapsed time tracking.

			Util.getElaspedTime();
		}
	}

	@Override
	public void execute() {
		int avgEncoderCount = driveBase.getAvgEncoder();

		Util.consoleLog();

		LCD.printLine(LCD_4, "Auto wheel encoder avg=%d", avgEncoderCount);

		// Use PID to determine the power applied. Should reduce power as we get close
		// to the target encoder value.

		if (pid == Pid.on) {
			elapsedTime = Util.getElaspedTime();

			power = pidController.calculate(avgEncoderCount, elapsedTime);

			// power = pidController.get();

			Util.consoleLog("avenc=%d  error=%.2f  power=%.3f  time=%.3f", avgEncoderCount, pidController.getError(),
					power, elapsedTime);
		} else
			Util.consoleLog("tgt=%d  act=%d", encoderCounts, Math.abs(avgEncoderCount));

		// Yaw angle is negative if robot veering left, positive if veering right when
		// going forward.

		if (heading == Heading.heading)
			yaw = RobotContainer.navx.getHeadingYaw();
		else
			yaw = RobotContainer.navx.getYaw();

		LCD.printLine(LCD_5, "yaw=%.2f", yaw);

		Util.consoleLog("yaw=%.2f  hdg=%.2f  hdgyaw=%.2f  curve=%.2f", yaw, RobotContainer.navx.getHeading(),
				RobotContainer.navx.getHeadingYaw(), Util.clampValue(-yaw * kSteeringGain, 1.0));

		// Note we invert sign on the angle because we want the robot to turn in the
		// opposite
		// direction than it is currently going to correct it. So a + angle says robot
		// is veering
		// right so we set the turn value to - because - is a turn left which corrects
		// our right
		// drift. kSteeringGain controls how aggressively we turn to stay on course.

		driveBase.curvatureDrive(power, Util.clampValue(-yaw * kSteeringGain, 1.0), false);

		// Util.consoleLog("lpwr=%.2f rpwr=%.2f", driveBase.getLeftPower(),
		// -driveBase.getRightPower());
		Util.consoleLog("lpwr=%.2f  rpwr=%.2f", driveBase.getLeftPower(), driveBase.getRightPower());

		iterations++;
	}

	@Override
	public void end(boolean interrupted) {
		Util.consoleLog("interrupted=%b", interrupted);

		if (stop == StopMotors.stop)
			driveBase.stop();

		int actualCount = Math.abs(driveBase.getAvgEncoder());

		Util.consoleLog("target counts=%d  actual count=%d  error=%.2f pct", encoderCounts, actualCount,
				((double) actualCount - encoderCounts) / (double) encoderCounts * 100.0);

		Util.consoleLog("distmeters=%.3f  poseX=%.3f", driveBase.getAvgEncoderDist(),
				driveBase.getOdometerPose().getX() - 1.2);

		Util.consoleLog("iterations=%d  elapsed time=%.3fs", iterations, Util.getElaspedTime(startTime));

		Util.consoleLog("end -----------------------------------------------------");
	}

	@Override
	public boolean isFinished() {
		return Math.abs(driveBase.getAvgEncoder()) >= encoderCounts;
	}

	public enum Brakes {
		off, on
	}

	public enum Pid {
		off, on
	}

	public enum Heading {
		angle, heading, totalAngle
	}

	public enum StopMotors {
		dontStop, stop
	}
}
