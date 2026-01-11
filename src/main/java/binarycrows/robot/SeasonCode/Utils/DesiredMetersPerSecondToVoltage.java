package binarycrows.robot.SeasonCode.Utils;

/*
 * Utility to use statistical analysis to approximate the voltage required for our motors to achieve a specific velocity.
 * Our data and regression calculations are in Desmos: https://www.desmos.com/calculator/o8uayiat4b
 * Note that maximum speed math has been included for reference but removed to help with acceleration.
 */
public class DesiredMetersPerSecondToVoltage {
    
    public static final double minSpeedMPS = 0.129641;
    public static final double correctionFactor = 0.01;
    //public static final double maxSpeedMPS = 4.37175;

    public static final double voltageAtMinSpeed = 0;
    //public static final double voltageAtMaxSpeed = 12.12027;

    public static double metersPerSecondToVoltageRegression(double mps) {
        return Math.abs(2.61031 * mps + minSpeedMPS - correctionFactor);
    }
    public static double metersPerSecondToVoltage(double desiredMetersPerSecond) {

        return Math.signum(desiredMetersPerSecond) * ( // Apply correct velocity direction. This is needed since the function is not even.
            (Math.abs(desiredMetersPerSecond) == 0) ? // If speed is less than the minimum (governed by static friction)...
            voltageAtMinSpeed : // ...return voltage at minimum speed
            /*(desiredMetersPerSecond > maxSpeedMPS) ? // If speed is greater than the maximum (governed by motor quality)...
            voltageAtMaxSpeed : // ..return voltage at maximum speed */
            metersPerSecondToVoltageRegression(Math.abs(desiredMetersPerSecond)) // Return the result of our regression line
        );
    }
}