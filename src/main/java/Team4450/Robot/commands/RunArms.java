package Team4450.Robot.commands;

import java.util.ArrayList;
import Team4450.Lib.Util;
import Team4450.Robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunArms extends CommandBase {
	private final ArrayList<Arm> arms;

	public RunArms(ArrayList<Arm> arms) {
		Util.consoleLog();

		this.arms = arms;

		for (Arm arm : arms)
			addRequirements(arm);
	}

	@Override
	public void initialize() {
		Util.consoleLog();
	}

	@Override
	public void execute() {
		double power = .25;

		for (Arm arm : arms)
			arm.setMotorPower(power);
	}

	@Override
	public void end(boolean interrupted) {
		Util.consoleLog("interrupted=%b", interrupted);

		for (Arm arm : arms)
			arm.stopMotor();
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
