package Team4450.Robot.subsystems;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Lib.SRXMagneticEncoderRelative.PIDRateType;
import Team4450.Robot.RobotContainer;
import Team4450.Robot.commands.Utility.NotifierCommand;
import static Team4450.Robot.Constants.*;

import Team4450.Lib.FunctionTracer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class hosts functions relating to communicating with the ShuffleBoard driver
 * station application. Primarily, it's periodic function handles the regular update
 * of the "LCL" panel's display of robot status information when the robot is active.
 */
public class ShuffleBoard extends SubsystemBase
{
    public int                  currentTab, numberOfTabs = 2;

    private NotifierCommand     updateCommand;
    private Notifier            notifier;

	public ShuffleBoard()
	{
        // We use a NotifierCommand to run the DS update process in a separate thread
        // from the main thread. We set that command as the default command for this
        // subsystem so the scheduler starts the command. After start, the notifier
        // runs all the time updating the DS every 25ms which is slightly slower than
        // the main thread update period.
        updateCommand = new NotifierCommand(this::updateDS, .025, "SB", this);

        this.setDefaultCommand(updateCommand);

		Util.consoleLog("ShuffleBoard created!");
	}

	// This method will be called once per scheduler run on the scheduler (main) thread. Only
    // used if not running the updateDS with the notifier.
	@Override
	public void periodic()
    {
        //updateDS();
    }

    public void updateDS()
	{    
        if (tracing) FunctionTracer.INSTANCE.enterFunction("ShuffleBoard.updateDS");

        LCD.printLine(LCD_3, "leftenc=%d  rightenc=%d", 
                      RobotContainer.driveBase.getLeftEncoder(), 
                      RobotContainer.driveBase.getRightEncoder());			
                
        LCD.printLine(LCD_4, "utilRY=%.3f  utilRX=%.3f",
                      RobotContainer.utilityPad.getRightYDS().getAsDouble(), 
                      RobotContainer.utilityPad.getRightX());
    
        LCD.printLine(LCD_7, "Lrpm=%d - Rrpm=%d  Lmax vel=%.3f - Rmax vel=%.3f", 
                      RobotContainer.driveBase.leftEncoder.getRPM(),
                      RobotContainer.driveBase.rightEncoder.getRPM(), 
                      RobotContainer.driveBase.leftEncoder.getMaxVelocity(PIDRateType.velocityMPS),
                      RobotContainer.driveBase.rightEncoder.getMaxVelocity(PIDRateType.velocityMPS));
      
        Pose2d pose = RobotContainer.driveBase.getOdometerPose();
      
        LCD.printLine(LCD_8, "pose x=%.1fm (lrot=%.2f)  y=%.1fm  deg=%.1f  yaw=%.1f", pose.getX(), 
                      RobotContainer.driveBase.leftEncoder.getRotations(), pose.getY(), pose.getRotation().getDegrees(),
                      RobotContainer.navx.getYaw());
                      
        // LCD.printLine(LCD_10, "yaw=%.1f tyaw=%.1f tyaw180=%.1f tangle=%.1f hdg=%.1f",
        //               RobotContainer.navx.getYaw(), RobotContainer.navx.getTotalYaw(), RobotContainer.navx.getTotalYaw180(),
        //               RobotContainer.navx.getTotalAngle(), RobotContainer.navx.getHeading());

        // LCD.printLine(LCD_10, "ltt=%d  labspos=%d  ldeg=%.1f  rtt=%d  rabspos=%d  rdeg=%.1f",
        //               RobotContainer.driveBase.leftEncoder.getTotalTicks(),
        //               RobotContainer.driveBase.leftEncoder.getAbsolutePosition(),
        //               SRXMagneticEncoderRelative.ticksToDegrees(RobotContainer.driveBase.leftEncoder.getAbsolutePosition()),
        //               RobotContainer.driveBase.rightEncoder.getTotalTicks(),
        //               RobotContainer.driveBase.rightEncoder.getAbsolutePosition(),
        //               RobotContainer.driveBase.rightEncoder.getAbsolutePositionDeg());

        // Required to seed getMaxVelocity calls.
        //RobotContainer.canCoder.getRPM();
        //RobotContainer.driveBase.leftEncoder.getRPM();

        // LCD.printLine(LCD_10, "ccpos=%d ccabspos=%d ccdeg=%.1f ccmax=%.1f  lpos=%d labspos=%d ldeg=%.1f lmax=%.1f",
        //               RobotContainer.canCoder.get(),
        //               RobotContainer.canCoder.getAbsolutePosition(),
        //               RobotContainer.canCoder.getAbsolutePositionDeg(),
        //               RobotContainer.canCoder.getMaxVelocity(CANCoder.PIDRateType.velocityMPS),
        //               RobotContainer.driveBase.leftEncoder.get(),
        //               RobotContainer.driveBase.leftEncoder.getAbsolutePosition(),
        //               RobotContainer.driveBase.leftEncoder.getAbsolutePositionDeg(),
        //               RobotContainer.driveBase.leftEncoder.getMaxVelocity(PIDRateType.velocityMPS));
            
        if (tracing) FunctionTracer.INSTANCE.exitFunction("ShuffleBoard.updateDS");
    }

    /**
     * Reset the shuffleboard indicators to disabled states.
     */
    public void resetLEDs()
    {
        // Notifier runs the reset function in a separate thread.
        notifier = new Notifier(this::resetLEDIndicators);
        notifier.startSingle(0);
    }

    private void resetLEDIndicators()
    {
        Util.consoleLog();
        
        SmartDashboard.putBoolean("Disabled", true);
        SmartDashboard.putBoolean("Auto Mode", false);
        SmartDashboard.putBoolean("Teleop Mode", false);
        SmartDashboard.putBoolean("FMS", DriverStation.isFMSAttached());
        SmartDashboard.putBoolean("Overload", false);
        SmartDashboard.putNumber("AirPressure", 0);
        SmartDashboard.putBoolean("AltDriveMode", false);
        SmartDashboard.putBoolean("SteeringAssist", false);
        SmartDashboard.putBoolean("Brake", false);
    }

    /**
     * Switch tab on shuffleboard display by rotating through the tabs.
     * @return The new tab index (0-based).
     */
    public int switchTab()
    {
        currentTab++;

        if (currentTab > (numberOfTabs - 1)) currentTab = 0;

        Util.consoleLog("%d", currentTab);

        Shuffleboard.selectTab(currentTab);

        return currentTab;
    }

    /**
     * Switch tab on shuffleboard display by tab name. Will create the tab if
     * it does not already exist.
     * @param tabName The name of the tab to select.
     * @return The selected tab object.
     */
    public ShuffleboardTab switchTab(String tabName)
    {
        Util.consoleLog("%s", tabName);

        return Shuffleboard.getTab(tabName);
    }
}
