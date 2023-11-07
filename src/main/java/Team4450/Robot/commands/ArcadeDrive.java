
package Team4450.Robot.commands;

import static Team4450.Robot.Constants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Lib.SRXMagneticEncoderRelative.PIDRateType;
import Team4450.Robot.RobotContainer;
import Team4450.Robot.subsystems.DriveBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Driving command that feeds target speed and rotation (% power) to the DriveBase.
 * Used for single stick arcade style driving.
 */
public class ArcadeDrive extends CommandBase 
{
  private final DriveBase 		  driveBase;
  
  private final DoubleSupplier	powerSupplier, rotationSupplier;
  private final BooleanSupplier turnInPlaceSupplier;

  private final double          steeringGain = .4;
  
  /**
   * Creates a new ArcadeDrive command.
   *
   * @param subsystem The subsystem used by this command.
   * @param speed The speed as % power -1.0 to +1.0. + is forward.
   * @param turnInPlace True to enable turn in place (only rotation valye applies).
   */
  public ArcadeDrive(DriveBase subsystem, DoubleSupplier power, DoubleSupplier rotation, BooleanSupplier turnInPlace) 
  {
    Util.consoleLog();
	  
	  driveBase = subsystem;
	  
	  // Use addRequirements() here to declare subsystem dependencies.
	  
	  addRequirements(this.driveBase);
	  
	  // Save references to DoubleSupplier objects so we can read them later in the
	  // execute method.
	  
	  powerSupplier = power;
    rotationSupplier = rotation;
    turnInPlaceSupplier = turnInPlace;
  }

  /**
   *  Called when the command is scheduled to run.
   *  NOTE: This command is set as the default for the drive base. That
   *  means it runs as long as no other command that uses the drive base
   *  runs. If another command runs, like shift gears for instance, this
   *  command will be interrupted and then rescheduled when shift gears
   *  is finished. That reschedule means initialize() is called again.
   *  When the robot is disabled this command is interrupted but retained
   *  by the scheduler. When reenabled scheduler will call initialize to
   *  restart running this command. Commands only cease to exist (deleted
   *  by scheduler) when thier isFinished method returns true. All commands
   *  work like this.
   */
  @Override
  public void initialize() 
  {
	  Util.consoleLog();
	  
	  driveBase.setMotorSafety(true); 	// Turn on watchdog.

    // 2018 post season testing showed this setting helps smooth out driving response.
    // Set here because auto programs may set their own rate. We combine this with
    // squared input on drive methods to try to reduce how jerky and touchy the 
    // robot can be.
      
    driveBase.SetCANTalonRampRate(TALON_RAMP_RATE);
  }

  /** 
   * Called every time the scheduler runs while the command is scheduled. Passes
   * the speed & rotation values provided by whatever double provider was passed
   * in the constructor to the drive base arcade drive function. The providers are
   * typically the joystick X & Y deflection values but can be any double provider.
   */
  @Override
  public void execute() 
  {
    double power = powerSupplier.getAsDouble(), rotation = rotationSupplier.getAsDouble();

    boolean turnInPlace = turnInPlaceSupplier.getAsBoolean();

    LCD.printLine(LCD_2, "power=%.3f  rot=%.3f tip=%b  (lpwr=%.3f) (rpwr=%.3f)   ", power, rotation,
                  turnInPlace, driveBase.getLeftPower(), driveBase.getRightPower());

    // Tank or Arcade Drive are default commanads for the DriveBase. When running in autonmous, the auto commands
    // require DriveBase, which preempts the default DriveBase command. However, if our auto code ends before end
    // of auto period, then the tank/arcade drive command resumes and is feeding drivebase during remainder of auto
    // period. This was not an issue until the joystick drift problem arose, so the resumption of a driving command
    // during auto had the robot driving randomly after our auto program completed. The if statment below fixes this.
    if (robot.isAutonomous()) return;
        
    if (turnInPlace)
      driveBase.curvatureDrive(0, rotation * steeringGain, true);
    else
	    driveBase.arcadeDrive(power, rotation * steeringGain, false);
  }
  
  /**
   *  Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) 
  {
	  Util.consoleLog("interrupted=%b", interrupted);
	  
	  driveBase.stop();
	  
	  driveBase.setMotorSafety(false); 	// Turn off watchdog.
  }

  /**
   *  Returns true when the command should end. Returning false means it never ends.
   */
  @Override
  public boolean isFinished() 
  {
    return false;
  }

  /**
   * Method present to allow TankDrive and ArcadeDrive to be instantiated into the
   * same command. Does nothing on ArcadeDrive.
   */
  public void toggleAlternateDrivingMode()
  {
  }
}
