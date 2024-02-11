package Team4450.Robot.commands.autonomous;

import static Team4450.Robot.Constants.*;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Robot.RobotContainer;
import Team4450.Robot.subsystems.DriveBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This is an example autonomous command using 4450 written auto driving
 * commands which are customized versions of the Wpilib motion profile based PID
 * commands.
 */
public class TestAuto2 extends CommandBase {
	private final DriveBase driveBase;

	private SequentialCommandGroup commands = null;
	private Command command = null;
	private Pose2d startingPose;

	/**
	 * Creates a new TestAuto2 autonomous command. This command demonstrates one
	 * possible structure for an autonomous command and shows the use of the
	 * autonomous driving support commands.
	 *
	 * @param driveBase
	 *            DriveBase subsystem used by this command to drive the robot.
	 */
	public TestAuto2(DriveBase driveBase, Pose2d startingPose) {
		Util.consoleLog();

		this.driveBase = driveBase;

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

		LCD.printLine(LCD_1, "Mode: Auto - TestAuto2 - All=%s, Location=%d, FMS=%b, msg=%s", alliance.name(), location,
				DriverStation.isFMSAttached(), gameMessage);

		// Reset wheel encoders.
		driveBase.resetEncodersWithDelay();

		// Set NavX yaw tracking to 0.
		RobotContainer.navx.resetYaw();

		// Set heading to initial angle (0 is robot pointed down the field) so
		// NavX class can track which way the robot is pointed all during the match.
		RobotContainer.navx.setHeading(startingPose.getRotation().getDegrees());

		// Target heading should be the same.
		RobotContainer.navx.setTargetHeading(startingPose.getRotation().getDegrees());

		// Set Talon ramp rate for smooth acceleration from stop. Determine by
		// observation.
		driveBase.SetCANTalonRampRate(1.0);

		// Reset odometry tracking with initial x,y position and heading (set above)
		// specific to this
		// auto routine. Robot must be placed in same starting location each time for
		// pose tracking
		// to work.
		driveBase.resetOdometer(startingPose, startingPose.getRotation().getDegrees());

		// Since a typical autonomous program consists of multiple actions, which are
		// commands
		// in this style of programming, we will create a list of commands for the
		// actions to
		// be taken in this auto program and add them to a sequential command list to be
		// executed one after the other until done.

		commands = new SequentialCommandGroup();

		// First action is to drive forward somedistance and stop with brakes on.

		command = new AutoDriveProfiled(driveBase, 2, AutoDrive.StopMotors.stop, AutoDrive.Brakes.on);

		commands.addCommands(command);

		// Next action is to rotate left 90.

		command = new AutoRotateProfiled(driveBase, -90);

		commands.addCommands(command);

		// Next action is to drive distance and stop with brakes on.

		command = new AutoDriveProfiled(driveBase, 2, AutoDrive.StopMotors.stop, AutoDrive.Brakes.on);

		commands.addCommands(command);

		// Now rotate to heading 0.

		command = new AutoRotateHdgProfiled(driveBase, 0);

		commands.addCommands(command);

		// Now drive a curve to 90 deg right.

		command = new AutoCurve(driveBase, .30, .20, 90, AutoDrive.StopMotors.stop, AutoDrive.Brakes.on,
				AutoDrive.Pid.on, AutoDrive.Heading.angle);

		commands.addCommands(command);

		// Launch autonomous command sequence.

		commands.schedule();
	}

	/**
	 * Called every time the scheduler runs while the command is scheduled. In this
	 * model, this command just idles while the Command Group we created runs on its
	 * own executing the steps (commands) of this Auto program.
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

		return !commands.isScheduled();
	}
}
