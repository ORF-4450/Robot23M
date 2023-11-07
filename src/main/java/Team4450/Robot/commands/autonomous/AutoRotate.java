package Team4450.Robot.commands.autonomous;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot.RobotContainer;
import Team4450.Robot.commands.autonomous.AutoDrive.Heading;
import Team4450.Robot.commands.autonomous.AutoDrive.Pid;
import Team4450.Robot.subsystems.DriveBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoRotate extends CommandBase
{
	private final DriveBase driveBase;

	private double			yaw, elapsedTime = 0, power, target, saveHeading; 
	private double			kP = .0075, kI = 0.0075, kD = 0, kTolerance = 1.5, startTime;
	private int				iterations;
	private Pid 			pid;
	private Heading 		heading;
	
	SynchronousPID			pidController = null;
	
	/**
	 * Creates a new AutoRotate command.
	 * 
	 * Auto rotate the specified target angle from where the robot is currently pointing or rotate
	 * to a target heading.
	 * 
	 * @param driveBase The DriveBase subsystem used by this command to drive the robot.
	 * @param power Max power for rotation. Power is always +.
	 * @param target Target angle (-left, +right) to rotate from robot current direction -180..+180, or 
	 * target heading (0..359) to rotate to from robot current heading. Target heading is always + and cannot be more 
	 * than 180 degrees away from current heading.
	 * @param pid Off for simple rotation, On use PID controller to manage the rotation slowing rotation as
	 * target is reached.
	 * @param heading Off target is an angle, On target is a heading.
	 * 
	 * Note: This routine is designed for tank drive and the P,I,D,tolerance values will likely need adjusting for each
	 * new drive base as gear ratios and wheel configuration may require different values to turn smoothly
	 * and accurately.
	 */
	public AutoRotate(DriveBase driveBase, 
					  double 	power, 
					  double 	target, 
					  Pid 		pid, 
					  Heading 	heading)
	{
		this.driveBase = driveBase;
		
		Util.consoleLog("pwr=%.2f  target=%.2f  pid=%s  hdg=%s", power, target, pid, heading);
		
		if (power <= 0) throw new IllegalArgumentException("power must be + non-zero");
			  
		this.power = power;
		this.target = target;
		this.pid = pid;
		this.heading = heading;
		
		Util.consoleLog("kP=%.3f  kI=%.3f", kP, kI);

		addRequirements(this.driveBase);
	}
	
	@Override
	public void initialize()
	{
        Util.consoleLog("pwr=%.2f  target=%.2f  pid=%s  hdg=%s", power, target, pid, heading);

		startTime = Util.timeStamp();

		// Try to prevent over rotation.
		
		driveBase.SetCANTalonBrakeMode(true);

		// Reset yaw to current robot direction or target heading.
		
		if (heading == Heading.heading) 
		{
			Util.checkRange(target, 0, 359, "target");
			
			RobotContainer.navx.setTargetHeading(target);
		}
		else
		{
			Util.checkRange(target, 180, "target");
			
			RobotContainer.navx.resetYaw(); //Wait(1, 1000);
		}
		
		if (pid == Pid.on)
		{
			// Use PID to control power as we turn slowing as we approach target heading.
			
			pidController = new SynchronousPID(kP, kI, kD);
			
			pidController.setOutputRange(-power , power);
			
			if (heading == Heading.heading)
				pidController.setSetpoint(0);		// We are trying to get the yaw to zero.
			else				
				pidController.setSetpoint(target);	// We are trying to get to the target yaw.
			
			// The PID class needs delta time between calls to calculate the I term.
			
			Util.getElaspedTime();
		}
		else if (heading == Heading.heading)		// Simple turn, full power until target heading reached.
		{
			yaw = RobotContainer.navx.getHeadingYaw();

			if (yaw > 0) power = power * -1;
		}
		else 							// Simple turn, full power until target angle reached.
		{
			yaw = RobotContainer.navx.getYaw();
			
			if (target < 0) power = power * -1;
		}
	}
	
	@Override
	public void execute() 
	{
		double	curve = 0;

		Util.consoleLog();
		
		saveHeading = RobotContainer.navx.getHeading();

		if (pid == Pid.on)
		{
			if (heading == Heading.heading)
				yaw = RobotContainer.navx.getHeadingYaw();
			else
				yaw = RobotContainer.navx.getYaw();
			
			elapsedTime = Util.getElaspedTime();
			
			// Our target is zero yaw so we determine the difference between
			// current yaw and target and perform the PID calculation which
			// results in the speed of turn, reducing power as the difference
			// approaches zero. So our turn should slow and not overshoot. If
			// it does, the PID controller will reverse power and turn it back.
			// This continues until the error is within tolerance.
			
			curve = pidController.calculate(yaw, elapsedTime);
			
			// When quick turn is true, first parameter is not used, power is fed to the
			// rate of turn parameter. PID controller takes care of the sign, that 
			// is the left/right direction of the turn.
			
			driveBase.curvatureDrive(0, curve, true);
			
			Util.consoleLog("curve=%.2f  hdg=%.2f  yaw=%.2f  err=%.2f  time=%.4f", curve, 
							 saveHeading, yaw, pidController.getError(), elapsedTime); 
		}
		else if (heading == Heading.heading)
		{
			driveBase.curvatureDrive(0, curve, true);
			
			yaw = RobotContainer.navx.getHeadingYaw();
			
			Util.consoleLog("curve=%.2f  yaw=%.2f  hdg=%.2f", curve, yaw, saveHeading);
		}
		else 
		{
			driveBase.curvatureDrive(0, curve, true);
			
			yaw = RobotContainer.navx.getYaw();
			
			Util.consoleLog("yaw=%.2f  hdg=%.2f", yaw, saveHeading);
		}

		iterations++;
	}
	
	@Override
	public void end(boolean interrupted) 
	{
		Util.consoleLog("interrupted=%b", interrupted);
		
        if (heading == Heading.heading)
    		Util.consoleLog("end loop: hdg=%.2f  hdgyaw=%.2f", saveHeading, yaw);
        else
            Util.consoleLog("end loop: hdg=%.2f  yaw=%.2f", saveHeading, yaw);
	
        driveBase.stop();
		
        //Util.consoleLog("after stop: hdg=%.2f  hdgyaw=%.2f  yaw=%.2f", RobotContainer.navx.getHeading(), 
        //                RobotContainer.navx.getHeadingYaw(), RobotContainer.navx.getYaw());
	
		// Wait for robot to stop moving. Note: this only works if motor stop requested.
        // while (RobotContainer.navx.getYawRate() > .99) 
        // {
        //     Util.consoleLog("yaw rate=%.2fd/s", RobotContainer.navx.getYawRate());
        //     Timer.delay(.010);
        // }
		
        Util.consoleLog("end: hdg=%.2f  hdgyaw=%.2f  yaw=%.2f", RobotContainer.navx.getHeading(), 
                        RobotContainer.navx.getHeadingYaw(), RobotContainer.navx.getYaw());
		
		Util.consoleLog("iterations=%d  elapsed time=%.3fs", iterations, Util.getElaspedTime(startTime));

		Util.consoleLog("end ----------------------------------------------------------");
	}
	
	@Override
	public boolean isFinished() 
	{
		if (pid == Pid.on)
			return pidController.onTarget(kTolerance);			
		else if (heading == Heading.heading)
			return Util.checkRange(yaw, 1.0);
		else 
			return Math.abs(yaw) >= Math.abs(target);
	}
}
