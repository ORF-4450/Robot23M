package Team4450.Robot.subsystems;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
	Talon motor;

	public Arm(int pwmPort) {
		Util.consoleLog("%d", pwmPort);

		motor = new Talon(pwmPort);
	}

	public void setMotorPower(double power) {
		Util.consoleLog("%.2f", power);

		motor.set(power);
	}

	public void stopMotor() {
		motor.stopMotor();
	}
}
