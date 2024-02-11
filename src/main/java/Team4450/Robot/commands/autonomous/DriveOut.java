package Team4450.Robot.commands.autonomous;

import static Team4450.Robot.Constants.*;

import Team4450.Lib.LCD;
import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.Util;
import Team4450.Robot.Robot;
import Team4450.Robot.RobotContainer;
import Team4450.Robot.subsystems.DriveBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Drive out of the starting area.
 */
public class DriveOut extends CommandBase {
	private final DriveBase driveBase;

	private SequentialCommandGroup commands = null;
	private Command command = null;
	private Pose2d startingPose;

	/**
	 * Creates a new DriveOut autonomous command.
	 *
	 * @param subsystem
	 *            The subsystem used by this command.
	 */
	public DriveOut(DriveBase subsystem, Pose2d startingPose) {
		Util.consoleLog("x=%.3f  y=%.3f  hdg=%.1f", startingPose.getX(), startingPose.getY(),
				startingPose.getRotation().getDegrees());

		driveBase = subsystem;

		this.startingPose = startingPose;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(this.driveBase);
	}

	/**
	 * Called when the command is initially scheduled. (non-Javadoc)
	 *
	 * @see edu.wpi.first.wpilibj2.command.Command#initialize()
	 */
	@Override
	public void initialize() {
		Util.consoleLog();

		driveBase.setMotorSafety(false); // Turn off watchdog.

		LCD.printLine(LCD_1, "Mode: Auto - DriveOut - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location,
				DriverStation.isFMSAttached(), gameMessage);

		// Reset wheel encoders.
		if (Robot.isReal())
			driveBase.resetEncodersWithDelay();
		else
			driveBase.resetSimEncodersWithDelay();

		// Set NavX yaw tracking to 0.
		RobotContainer.navx.resetYaw();

		// Set heading to initial angle (0 is robot pointed down the field) so
		// NavX class can track which way the robot is pointed during the match.
		RobotContainer.navx.setHeading(startingPose.getRotation().getDegrees());

		// Target heading should be the same.
		RobotContainer.navx.setTargetHeading(startingPose.getRotation().getDegrees());

		// Set Talon ramp rate for smooth acceleration from stop. Determine by
		// observation.
		driveBase.SetCANTalonRampRate(1.0);

		// Reset odometry tracking with initial x,y position and heading (passed at
		// constructor).
		// Robot must be placed in same starting location each time for pose tracking
		// to work.
		driveBase.resetOdometer(startingPose, startingPose.getRotation().getDegrees());

		// Since a typical autonomous program consists of multiple actions, which are
		// commands
		// in this style of programming, we will create a list of commands for the
		// actions to
		// be taken in this auto program and add them to a sequential command list to be
		// executed one after the other until done.

		commands = new SequentialCommandGroup();

		// Rotate left 90 degrees.

		command = new AutoRotate(driveBase, .30, -90, AutoDrive.Pid.on, AutoDrive.Heading.angle);

		// commands.addCommands(command);

		// Drive forward some encoder counts and stop with brakes on. We express the
		// target distance in feet and convert to ticks.

		command = new AutoDrive(driveBase, .30,
				SRXMagneticEncoderRelative.getTicksForDistance(3.5, DRIVE_WHEEL_DIAMETER), AutoDrive.StopMotors.stop,
				AutoDrive.Brakes.on, AutoDrive.Pid.on, AutoDrive.Heading.angle);

		commands.addCommands(command);

		// Rotate right 90 degrees.

		command = new AutoRotate(driveBase, .30, 90, AutoDrive.Pid.on, AutoDrive.Heading.angle);

		// commands.addCommands(command);

		// Drive forward again.

		command = new AutoDrive(driveBase, .30,
				SRXMagneticEncoderRelative.getTicksForDistance(3.5, DRIVE_WHEEL_DIAMETER), AutoDrive.StopMotors.stop,
				AutoDrive.Brakes.on, AutoDrive.Pid.on, AutoDrive.Heading.angle);

		// commands.addCommands(command);

		// Launch autonomous command sequence.

		commands.schedule();
	}

	/**
	 * Called every time the scheduler runs while the command is scheduled.
	 */
	@Override
	public void execute() {
	}

	/**
	 * Called when the command ends or is interrupted.
	 */
	@Override
	public void end(boolean interrupted) {
		Util.consoleLog("interrupted=%b", interrupted);

		driveBase.stop();

		Util.consoleLog("final heading=%.2f  Radians=%.2f", RobotContainer.navx.getHeading(),
				RobotContainer.navx.getHeadingR());
		Util.consoleLog("end ---------------------------------------------------------------");
	}

	/**
	 * Returns true when this command should end. That should be when all the
	 * commands in the command list have finished.
	 */
	@Override
	public boolean isFinished() {
		// Note: commands.isFinished() will not work to detect the end of the command
		// list
		// due to how FIRST coded the SquentialCommandGroup class.

		if (commands == null)
			return true;
		else
			return !commands.isScheduled();
	}
}
