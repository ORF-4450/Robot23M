package Team4450.Robot.commands;

import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import Team4450.Robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveArm extends CommandBase {
	private final Arm arm;

	private final DoubleSupplier armSupplier;

	public DriveArm(Arm arm, DoubleSupplier armSupplier) {
		Util.consoleLog();

		this.arm = arm;
		this.armSupplier = armSupplier;

		addRequirements(arm);
	}

	@Override
	public void initialize() {
		Util.consoleLog();
	}

	@Override
	public void execute() {
		double power = deadband(armSupplier.getAsDouble(), .05);

		arm.setMotorPower(power);
	}

	@Override
	public void end(boolean interrupted) {
		Util.consoleLog("interrupted=%b", interrupted);
	}

	/**
	 * Returns true when the command should end. Returning false means it never
	 * ends.
	 */
	@Override
	public boolean isFinished() {
		return false;
	}

	private static double deadband(double value, double deadband) {
		return Math.abs(value) > deadband ? value : 0.0;
	}
}
