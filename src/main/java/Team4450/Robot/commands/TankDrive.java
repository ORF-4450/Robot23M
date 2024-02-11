
package Team4450.Robot.commands;

import static Team4450.Robot.Constants.*;

import java.util.function.DoubleSupplier;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Robot.RobotContainer;
import Team4450.Robot.subsystems.DriveBase;
import Team4450.Lib.FunctionTracer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Driving command that feeds target left/right speeds (% power) to the
 * DriveBase. Used for two stick tank style driving.
 */
public class TankDrive extends CommandBase {
	private final DriveBase driveBase;

	private final DoubleSupplier leftPower, rightPower;

	private boolean altDriveMode, steeringAssistMode;
	private double kPowerGain = 1.0; // This is used to tone down joystick response.
	private double kMaxPower = .75; // Max joystick value to limit robot speed.

	/**
	 * Creates a new TankDrive command.
	 *
	 * @param subsystem
	 *            The subsystem used by this command.
	 * @param leftSpeed
	 *            The speed as % power -1.0 to +1.0. + is forward.
	 * @param rightSpeed
	 *            The speed as % power -1.0 to +1.0. + is forward.
	 */
	public TankDrive(DriveBase subsystem, DoubleSupplier leftPower, DoubleSupplier rightPower) {
		Util.consoleLog();

		driveBase = subsystem;

		// Use addRequirements() here to declare subsystem dependencies.

		addRequirements(this.driveBase);

		// Save references to DoubleSupplier objects so we can read them later in the
		// execute method.

		this.leftPower = leftPower;
		this.rightPower = rightPower;
	}

	/**
	 * Called when the command is scheduled to run. NOTE: This command is set as the
	 * default for the drive base. That means it runs as long as no other command
	 * that uses the drive base runs. If another command runs, like shift gears for
	 * instance, this command will be interrupted and then rescheduled when shift
	 * gears is finished. That reschedule means initialize() is called again. So it
	 * is important to realize that while this command class exists for the entire
	 * run of teleop, it stops when it is preempted by another command and then when
	 * rescheduled initialize will be called again and then execute resumes being
	 * repeatedly called. When the robot is disabled this command is interrupted but
	 * retained by the scheduler. When reenabled scheduler will call initialize to
	 * restart running this command. Commands only cease to exist (deleted by
	 * scheduler) when thier isFinished method returns true. All commands work like
	 * this.
	 */
	@Override
	public void initialize() {
		Util.consoleLog();

		driveBase.setMotorSafety(true); // Turn on watchdog.

		// 2018 post season testing showed this setting helps smooth out driving
		// response.
		// Set here because auto programs may set their own rate. We combine this with
		// squared input on drive methods to try to reduce how jerky and touchy the
		// robot can be.

		driveBase.SetCANTalonRampRate(TALON_RAMP_RATE);

		// driveBase.setPowerDeadBand(.35);
	}

	/**
	 * Called every time the scheduler runs while the command is scheduled. Passes
	 * the left/right speed values provided by whatever double provider was passed
	 * in the constructor, to the drive base tank drive function. The providers are
	 * typically the joystick Y deflection values but can be any double provider.
	 */
	@Override
	public void execute() {
		if (tracing)
			FunctionTracer.INSTANCE.enterFunction("TankDrive.execute");

		double leftY = leftPower.getAsDouble() * kPowerGain, rightY = rightPower.getAsDouble() * kPowerGain, angle;

		// Limit joystick max input value. Gain can reduce power but does it over the
		// entire range of JS values.
		// This can mean really sluggish behavior at small input. Limiting max caps max
		// robot speed but leaves the
		// lower speeds unchanged. Note that the drive base has limiting functions as
		// well that seek to reduce
		// the reponsiveness of the robot through delays in application of power to make
		// driving more manageable.
		// So joystick response is customizable in several locations. Do not use squared
		// inputs if you apply a
		// max power value. Squared inputs expect power range to be 0..1.
		leftY = Util.clampValue(leftY, kMaxPower);
		rightY = Util.clampValue(rightY, kMaxPower);

		LCD.printLine(LCD_2, "leftY=%.3f (%.3f)  rightY=%.3f (%.3f)", leftY, driveBase.getLeftPower(), rightY,
				driveBase.getRightPower());

		// Tank or Arcade Drive are default commanads for the DriveBase. When running in
		// autonmous, the auto commands
		// require DriveBase, which preempts the default DriveBase command. However, if
		// our auto code ends before end
		// of auto period, then the tank/arcade drive command resumes and is feeding
		// drivebase during remainder of auto
		// period. This was not an issue until the joystick drift problem arose, so the
		// resumption of a driving command
		// during auto had the robot driving randomly after our auto program completed.
		// The if statment below fixes this.
		if (robot.isAutonomous())
			return;

		if (altDriveMode) { // normal tank with straight drive assist when sticks within 10% of each other
							// and
							// right stick power is greater than 50%.
			if (isLeftRightEqual(leftY, rightY, 10) && Math.abs(rightY) > .50) {
				// Reset angle measurement when entering this code first time after mode is
				// enabled.
				if (!steeringAssistMode)
					RobotContainer.navx.resetYaw();

				// Angle is negative if robot veering left, positive if veering right when going
				// forward.
				// It is opposite when going backward.

				angle = (int) RobotContainer.navx.getYaw();

				// LCD.printLine(5, "angle=%d", angle);

				// Invert angle for backwards movement.

				if (rightY < 0)
					angle = -angle;

				// Util.consoleLog("angle=%d", angle);

				// Note we invert sign on the angle because we want the robot to turn in the
				// opposite
				// direction than it is currently going to correct it. So a + angle says robot
				// is veering
				// right so we set the turn value to - because - is a turn left which corrects
				// our right
				// drift.

				driveBase.curvatureDrive(rightY, -angle * STEERING_ASSIST_GAIN, false);

				steeringAssistMode = true;
			} else {
				steeringAssistMode = false;
				driveBase.tankDrive(leftY, rightY, false); // Normal tank drive.
			}

			SmartDashboard.putBoolean("SteeringAssist", steeringAssistMode);
		} else
			driveBase.tankDrive(leftY, rightY, true); // Normal tank drive.
		// driveBase.tankDriveLimited(leftY, rightY); // Normal tank drive acceleration
		// limited.

		if (tracing)
			FunctionTracer.INSTANCE.exitFunction("TankDrive.execute");
	}

	private boolean isLeftRightEqual(double left, double right, double percent) {
		if (Math.abs(left - right) <= (1 * (percent / 100)))
			return true;

		return false;
	}

	/**
	 * Toggles alternate drive mode.
	 */
	public void toggleAlternateDrivingMode() {
		Util.consoleLog("%b", altDriveMode);

		altDriveMode = !altDriveMode;

		steeringAssistMode = false;

		SmartDashboard.putBoolean("AltDriveMode", altDriveMode);
	}

	/**
	 * Called once the command ends or is interrupted.
	 */
	@Override
	public void end(boolean interrupted) {
		Util.consoleLog("interrupted=%b", interrupted);

		driveBase.stop();

		driveBase.setMotorSafety(false); // Turn off watchdog.
	}

	/**
	 * Returns true when the command should end. Returning false means it never
	 * ends.
	 */
	@Override
	public boolean isFinished() {
		return false;
	}
}
