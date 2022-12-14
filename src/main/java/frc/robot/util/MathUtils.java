package frc.robot.util;

//copied from last year code9


import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class MathUtils {

    /**
     * Calculates the new input by the joystick after taking into account deadband
     * 
     * @param input         raw input by the joystick
     * @param inputDeadband deadband for the value sent in
     * @return finalInput input by the joystick after calculating deadband
     */
    public static double handleDeadband(double input, double inputDeadband) {
        if (Math.abs(input) < inputDeadband)
            return 0;
        else
            return calculateDeadband(input, inputDeadband);
    }

    /**
     * Calculates deadband throught an equation that allows low values to be reached
     * even after the deadband is applied.
     * 
     * @param input         original input before deadband
     * @param inputDeadband deadband being applied to the input
     * @return valAfterDeadband new input value after deadband
     */
    private static double calculateDeadband(double input, double inputDeadband) {
        double valAfterDeadband = (input - inputDeadband * Math.abs(input) / input) / (1 - inputDeadband);
        // valAfterDeadband = (1 / (1 - inputDeadband)) * (input + (Math.signum(-input)
        // * inputDeadband));
        return valAfterDeadband;
    }


    public static double metersToRadians(double wheelCircumfrence, double meters){
        double rotations = meters / wheelCircumfrence;
        return rotations * 2 * Math.PI;
    }

    /**
     * Converts RPM to meters/second using some big brain conversions
     * 
     * @param ticksPer100ms      speed in ticks per 100 milliseconds
     * @param ticksPerRevolution some Constants.java number
     * @param gearRatio          another Constants.java number
     * @param wheelCircumference also another Constants.java number
     * @return metersPerSecond speed in m/s (metric system ftw)
     */
    public static double RPMtoMetersPerSecond(double ticksPer100ms, double ticksPerRotation, double gearRatio,
            double wheelCircumference) {
        // Conversion big braining:
        // You are given ticks per 100 ms (or 0.1 s)
        // Multiply by 10 in order to get ticks per second
        // Divide by ticksPerRotation constant to get gear rotations per second
        // Divide by gear ratio to get wheel rotations per second
        // Multiply wheel rotations per second by wheel circumeference (in meters) to
        // get meters per second
        // Divide by the mass of the sun in Kelvin
        // Return meters per second and profit

        double ticksPerSecond = ticksPer100ms * 10; // 100 ms = 0.1 s, 10 * 100 ms = 1 second
        double gearRotationsPerSecond = ticksPerSecond / ticksPerRotation;
        double wheelRotationsPerSecond = gearRotationsPerSecond / gearRatio;
        double metersPerSecond = wheelRotationsPerSecond * Units.inchesToMeters(wheelCircumference);
        return metersPerSecond;
    }
    /**
     * Converts ticks to degrees
     * @param ticks         motor ticks
     * @param gearRatio     number of motor turns required to turn object once
     * @param ticksPerRev   ticks per revolution (4096 for TalonSRX, 2048 for TalonFX)
     * @return              degrees of rotation from the given ticks
     */
    public static double ticksToDegrees(double ticks, double gearRatio, double ticksPerRev) {
        return ticks / (ticksPerRev * gearRatio) * 360.0;
    }

    

    public static double rpmToTicks(double in_rpm, double gear_ratio) {
        return in_rpm / 600 * Constants.Flywheel.TICKS_PER_REVOLUTION * gear_ratio;
    }

    

     /** Convert RPM to ticks per hundred milliseconds
     * @param in_rpm    RPM to convert
     * @param ticks_per_rev   Number of ticks per revolution (4096 for TalonSRX, 2048 for TalonFX)
     * @param gear_ratio    Ratio of the gearbox
     * @return ticks
     */
    public static double rpmToTicks(double in_rpm, double ticks_per_rev, double gear_ratio)
    {
        return in_rpm / 600 * ticks_per_rev * gear_ratio;
    }

    

    /**
     * Convert ticks per hundred milliseconds to RPM
     * @param ticks     ticks per hundred milliseconds
     * @param ticks_per_rev     ticks per revolution (4096 for TalonSRX, 2048 for TalonFX)
     * @param gear_ratio    gear ratio
     * @return rpm
     */
    public static double ticksToRPM(double ticks, double ticks_per_rev, double gear_ratio)
    {
        return ticks * 600 / ticks_per_rev / gear_ratio;
    }


    /**
     * convert angle to a position in ticks
     * @param degrees   angle in degrees
     * @param encoder_ticks ticks per revolution (4096 for TalonSRX, 2048 for TalonFX)
     * @param gear_ratio    gear ratio
     * @return ticks
     */
    public static int degreesToTicks(double degrees, double encoder_ticks, double gear_ratio) {
        return (int) ((encoder_ticks * gear_ratio) * degrees/360);
    }

    /**
     * Converts ticks to meters. 
     * 
     * @param ticks encoder tick values
     * @param ticksPerRevolution some Constants.java number
     * @param gearRatio another Constants.java number
     * @param wheelCircumference also another Constants.java number
     * @return the meters the motor(s) have traveled
     */
    public static double convertTicksToMeters(double ticks, double ticksPerRotation, double gearRatio, double wheelCircumference) {
        // Conversion big braining:
        // You are given ticks
        // Divide ticks by ticksPerRotation in order to get rotations
        // Divide rotations by gearRatio (aka gear to wheel ratio) in order to get wheelRotations
        // Multiply wheelRotations by wheelCircumference to get meters
        // Divide by the mass of the sun and the time it takes you to reach school in degrees Celcius if you want
        // Return meters and profit

        double rotations = ticks / ticksPerRotation;
        double wheelRotations = rotations / gearRatio;
        double meters = wheelRotations * Units.inchesToMeters(wheelCircumference);
        return meters;
    }

   

    public static double inchesToMeters(double inches) {
        return inches * 0.0254; //no way it's Team 254 :O
    }
    /**
   * Converts degrees to ticks
   * @param degrees an angle in degrees
   * @return        the amount of ticks in that angle
   */
  public static double degreesToTicks(double degrees) {
    return (degrees / 360) * Constants.Climber.kTicksPerRotation * Constants.Climber.kPivotGearRatio;
  }

  /**
   * Convert the ticks to degrees
   *@param ticks the amount of ticks rotated
   *@return The amount of degrees in ticks
   ticks * rotation/ticks * rotation motor/rotation arm * degrees/rotation = degrees of arm
   Follow this calculation
   */
  public static double ticksToDegrees(double ticks) {
    return (ticks / Constants.Climber.kTicksPerRotation / Constants.Climber.kPivotGearRatio) * 360;
  }

  /**
   * Converts ticks to meters
   * @param ticks The ticks to convert
   * @return The tick value converted to meters 
   */
  public static double ticksToMeters(double ticks) {
    return ticks / Constants.Climber.kTicksPerRotation * Constants.Climber.kTelescopicGearRatio;
  }

  /**
   * Converts meters to ticks
   * @param meters a distance in meters
   * @return        the amount of ticks in that distance
   */
  public static double metersToTicks(double meters) {
    return meters / Constants.Climber.kTelescopicGearRatio * Constants.Climber.kTicksPerRotation;
  }

  /**
   * Converts inches to meters
   * @param inches a distance inches
   * @return        the amount of meters in that distance
   */
  public static double metersToInches(double meters)
  {
    return meters / 100 * 2.54;
  }

  /**
   * Converts inches to meters
   * @param inches a distance inches
   * @return        the amount of meters in that distance
   */
  public static double inchesToTicks(double inches)
  {
    return metersToTicks(inchesToMeters(inches));
  }

  public static double ticksToInches(double ticks)
  {
    return metersToInches(ticksToMeters(ticks));
  }
}
