// Copyright (c) FIRST and other WPILib contributors.

package Team4450.Robot.subsystems;

import Team4450.Lib.Util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

/**
 * Utiity class that wraps CTRE CANdle LED controller.
 */
public class Candle extends SubsystemBase {

	private CANdle candle;
	private int numLed = 18;
	private double speed = .5, brightness = 1;
	private LEDStripType stripType = LEDStripType.RGB;

	public enum AnimationTypes {
		ColorFlow, Fire, Larson, Rainbow, RgbFade, SingleFade, Strobe, Twinkle, TwinkleOff, Off
	}

	/**
	 * Create a Candle class for CANdle device specifying the led strip attached to
	 * the device. All othere settings default.
	 *
	 * @param canId
	 *            The CAN id for the device.
	 * @param stripType
	 *            The Led strip type attached to the CANdle.
	 * @param numLed
	 *            Number of leds in the strip.
	 */
	public Candle(int canId, LEDStripType stripType, int numLed) {
		this(canId);

		setLedStripType(stripType);
		setNumLeds(numLed);
	}

	/**
	 * Create a candle class for CANdle device with default settings,
	 *
	 * @param canId
	 *            The CAN id for the device.
	 */
	public Candle(int canId) {
		candle = new CANdle(canId);

		CANdleConfiguration config = new CANdleConfiguration();

		config.stripType = stripType;
		config.brightnessScalar = brightness;
		config.statusLedOffWhenActive = true;
		config.disableWhenLOS = false;

		candle.configAllSettings(config);

		Util.consoleLog("Candle created (%d)", canId);
	}

	/**
	 * Set animation speed.
	 *
	 * @param speed
	 *            The speed of the animation in seconds.
	 */
	public void setSpeed(double speed) {
		this.speed = speed;
	}

	/**
	 * Set led brightness.
	 *
	 * @param brightness
	 *            Brightness value 0-1.
	 */
	public void setBrightness(double brightness) {
		this.brightness = brightness;
		candle.configBrightnessScalar(brightness);
	}

	/**
	 * Set the number of leds in the attached led strip.
	 *
	 * @param numLed
	 *            Number of leds.
	 */
	public void setNumLeds(int numLed) {
		this.numLed = numLed;
	}

	/**
	 * Set all leds to the color specified.
	 *
	 * @param red
	 *            Red color value 0-255.
	 * @param green
	 *            Green color value 0-255.
	 * @param blue
	 *            Blue color value 0-255.
	 */
	public void setLeds(int red, int green, int blue) {
		candle.setLEDs(red, green, blue);
	}

	/**
	 * Set all leds to the color specified.
	 *
	 * @param color
	 *            WPILib color value.
	 */
	public void setColor(Color color) {
		setLeds((int) color.red, (int) color.blue, (int) color.green);
	}

	/**
	 * Sets the type of led strip attached to the CANdle.
	 *
	 * @param stripType
	 *            Type of led strip.
	 */
	public void setLedStripType(LEDStripType stripType) {
		this.stripType = stripType;
		candle.configLEDType(stripType, 0);
	}

	/**
	 * Sets one of the built in animations to be displayed on the CANdle and led
	 * strip (if attached).
	 *
	 * @param animation
	 *            The desired animation.
	 */
	public void setAnimation(AnimationTypes animation) {
		Animation newAnimation = null;

		switch (animation) {
			case ColorFlow :
				newAnimation = new ColorFlowAnimation(128, 20, 70, 0, 0.7, numLed, Direction.Forward);
				break;
			case Fire :
				newAnimation = new FireAnimation(0.5, 0.7, numLed, 0.7, 0.5);
				break;
			case Larson :
				newAnimation = new LarsonAnimation(0, 255, 46, 0, 1, numLed, BounceMode.Front, 3);
				break;
			case Rainbow :
				newAnimation = new RainbowAnimation(1, 0.1, numLed);
				break;
			case RgbFade :
				newAnimation = new RgbFadeAnimation(0.7, 0.4, numLed);
				break;
			case SingleFade :
				newAnimation = new SingleFadeAnimation(50, 2, 200, 0, 0.5, numLed);
				break;
			case Strobe :
				newAnimation = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, numLed);
				break;
			case Twinkle :
				newAnimation = new TwinkleAnimation(30, 70, 60, 0, 0.4, numLed, TwinklePercent.Percent6);
				break;
			case TwinkleOff :
				newAnimation = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, numLed, TwinkleOffPercent.Percent100);
				break;
			case Off :
				newAnimation = null;
				break;
		}

		candle.animate(newAnimation);

		if (newAnimation == null)
			Util.consoleLog("Off");
		else
			Util.consoleLog(newAnimation.toString());
	}

}
