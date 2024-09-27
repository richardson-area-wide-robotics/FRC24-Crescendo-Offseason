package frc.utils;

public class JoystickUtils {
  public static double squareAxis(double input) {
    return (input * input) * Math.signum(input);
  }
}