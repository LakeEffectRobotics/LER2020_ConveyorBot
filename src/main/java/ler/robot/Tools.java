package ler.robot;

import ler.robot.subsystems.Gyro;

/**
 * Various utility functions.
 */
public class Tools {
    static final double DEADZONE = 0.15;

    /**
     * Halil Drive.
     * @param speed Raw input speed
     * @return Speed passed through function
     */
    public static double getAdaptedSpeed(double speed) {
        // Using a logisitic  function *** FUNCTION HAS BEEN FIXED ***
        //Alot harder to control in mid range but much nicer in low and high range.
        //double out = ((1.01339285092)/(1+Math.pow(Math.E,(-10*(speed-0.5)))))-0.5067;
        //2019 function: nicer in mid range worse control in low and high.
        
        if(Math.abs(speed)<DEADZONE){
            speed = 0;
        }
  
        double out = (0.5 * (Math.sin(Math.PI * speed - Math.PI / 2)) + 0.5);
        return out * (speed > 0 ? 1 : -1);
    }

    /**
     * Get number, adjusted to not exceed range.
     * 
     * @param value Number to fit
     * @param min Min possible result
     * @param max Max possible result
     * @return min if value&lt;min, max if value&gt;max, else value
     */
    public static double fitToRange(double value, double min, double max) {
        value = value < min ? min : value;
        value = value > max ? max : value;
        return value;
    }

    /** 
     * Ensure that value is further from 0 than +/- min.
     * 
     * @param value The value to check against
     * @param min The closest output to 0
     * @return <code>IF |min| > |value| THEN +/- min ELSE value </code>
     */
    public static double setAbsoluteMinimum(double value, double min) {
		value = (value < min && value > 0) ? min : value;
		value = (value > -min && value < 0) ? -min : value;
		return value;
    }
    
    /**
     * Get the equivalent (<360) agnle for an absolute angle.
     * <p> This is used as the gyro may have >360 values after driving for some time
     * <p> NOTE: This documentation may not be accurate the function's exact behaviour
     * 
     * @param gyro The {@link Gyro} subsystem
     * @param targetAngle Target angle
     * @return The angle to robot should turn towards
     */ 
    public static double closestEquivalentAngle(Gyro gyro, double targetAngle) {
		double t = targetAngle;
		double c = gyro.getAbsoluteAngle();
		boolean b = true;
		while(b){
			if(Math.abs(c - t) > Math.abs(c - (t + 360))){
	    		t += 360;
	    	} else if(Math.abs(c - t) > Math.abs(c - (t - 360))){
				t -= 360;
			} else {
				b = false;
			}
		}
		return t;
	}
}