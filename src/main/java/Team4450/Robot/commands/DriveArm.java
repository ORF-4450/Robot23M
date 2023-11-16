package Team4450.Robot.commands;

import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import Team4450.Robot.subsystems.Arm;
import static Team4450.Robot.Constants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveArm extends CommandBase 
{
    private final Arm               arm;
    private final XboxController    controller;

    private final DoubleSupplier armSupplier;

    public DriveArm(Arm arm, DoubleSupplier armSupplier, XboxController controller)
    {
        Util.consoleLog();

        this.arm = arm;
        this.armSupplier = armSupplier;
        this.controller = controller;

        addRequirements(arm);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();
    }
    
    @Override
    public void execute()
    {
        double power = deadband(armSupplier.getAsDouble(), .05);

        arm.setMotorPower(power);
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);
    }

    private static double deadband(double value, double deadband) 
    {
        return Math.abs(value) > deadband ? value : 0.0;
    }
}

