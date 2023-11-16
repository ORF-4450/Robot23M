
package Team4450.Robot;

import java.util.Properties;

import Team4450.Lib.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
	public static String		PROGRAM_NAME = "ORF23M-11.15.23-1";

	public static Robot			robot;

	public static Properties	robotProperties;
	  
	public static boolean		isClone = false, isComp = false, tracing = false;
	    	
	public static DriverStation.Alliance	alliance;
	public static int                       location, matchNumber;
	public static String					eventName, gameMessage;
	    
	// Drive motor controller port assignments.
	public static final int		LF_TALON = 1, LR_TALON = 2, RF_TALON = 3, RR_TALON = 4;

	// Other motor controller port assignments
	
	// GamePad port assignments.
	public static final int		DRIVER_PAD = 0, UTILITY_PAD = 1;

	// Pneumatic valve controller port assignments.
	public static final int		COMPRESSOR = 0;
	//public static final int		PICKUP_VALVE = 0;			// 0-1

	// Digital Input port assignments. Encoder takes 2 ports.
	  
	// Analog Input port assignments.
    public static final int		PRESSURE_SENSOR = 1;

    public static final double	TALON_RAMP_RATE = 1.0;			// Takes 1 sec for full power to be applied.
                                                                // Tried going above 1.0 but behavior became
                                                                // sketchy...
	public static final double  DRIVE_WHEEL_DIAMETER = 6.20;	// Inches.
	public static final double	STEERING_ASSIST_GAIN = .05;
    public static final double	TRACK_WIDTH = 30;				// Inches. 
	public static final double	GEAR_RATIO = 12.6;				// Overall gear ratio, motor rotations to one
																// wheel rotation.
	public static final double	ROBOT_WEIGHT = 140;				// Pounds.
	
	// LCD display line number constants showing class where the line is set.
	public static final int		LCD_1 = 1;	    // Robot, Auto Commands.
	public static final int		LCD_2 = 2;	    // TankDrive, ArcadeDrive command.
	public static final int		LCD_3 = 3;	    // ShuffleBoard subsystem.
	public static final int		LCD_4 = 4;	    // ShuffleBoard subsystem.
	public static final int		LCD_5 = 5;	    // ShuffleBoard subsystem.
	public static final int		LCD_7 = 7;	    // ShuffleBoard subsystem.
	public static final int		LCD_8 = 8;	    // ShuffleBoard subsystem.
	public static final int		LCD_9 = 9;	    // ShuffleBoard subsystem.
	public static final int		LCD_10 = 10;	// ShuffleBoard subsystem.

	// Default starting field position in meters & degrees for pose tracking. For full field lower left corner.
	public static final Pose2d	DEFAULT_STARTING_POSE = new Pose2d(1.2, 0.5, new Rotation2d(Math.toRadians(0)));

    // Use these values in PathWeaver for speed and acceleration.
    // Robot will go faster than this, more like 3 mps but this value tones down autonomous speed.

    public static final double  MAX_WHEEL_SPEED = 2.0;     // Meters per second.
    public static final double  MAX_WHEEL_ACCEL = 1.0;     // Meters per second per second.
    
    // Estimated by eyeball observation. Needs to be estimated each new robot.

    public static final double  MAX_ROTATIONAL_VEL = 70;    // Degrees per second.
    public static final double  MAX_ROTATIONAL_ACCEL = 70;  // Degrees per second per second.

    // Drive base characterization results. These values from 2021 as placeholders until 202?
	// characterization is done.

    public static final double  TRACK_WIDTH_C = Util.inchesToMeters(TRACK_WIDTH);	// Meters.

    public static final double  DB_KS = 1.74;
    public static final double  DB_KV = 1.8;
    public static final double  DB_KA = .422;

    public static final double  DB_POSITIONAL_KP = .0688; 
    public static final double  DB_POSITIONAL_KD = 36.5; 
    public static final double  DB_VELOCITY_KP = .12;  
    public static final double  DB_VELOCITY_KD = 0.0;
}
