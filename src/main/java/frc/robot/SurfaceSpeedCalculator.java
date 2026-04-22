package frc.robot;

public class SurfaceSpeedCalculator {

  /**
   * Converts angular velocity (rotations per second) to surface speed.
   *
   * @param angularVelocity rotations per second
   * @param wheelDiameter wheel diameter (inches, meters, etc.)
   * @return surface speed in the same linear units per second as wheelDiameter
   */
  public static double angularVelocityToSurfaceSpeed(double angularVelocity, double wheelDiameter) {
    return angularVelocity * (Math.PI * wheelDiameter);
  }

  /**
   * Converts surface speed to angular velocity (rotations per second).
   *
   * @param surfaceSpeed linear speed (inches per second, meters per second, etc.)
   * @param wheelDiameter wheel diameter (inches, meters, etc.)
   * @return angular velocity in rotations per second
   */
  public static double surfaceSpeedToAngularVelocity(double surfaceSpeed, double wheelDiameter) {
    return surfaceSpeed / (Math.PI * wheelDiameter);
  }

  /**
   * Given the angular velocity of one wheel and the diameters of two wheels, calculates the angular
   * velocity needed for the second wheel to match the surface speed of the first.
   *
   * @param angularVelocityA rotations per second of wheel A
   * @param diameterA diameter of wheel A
   * @param diameterB diameter of wheel B
   * @return rotations per second of wheel B
   */
  public static double getVelocityForMatchingSurfaceSpeed(
      double angularVelocityA, double diameterA, double diameterB) {
    double surfaceSpeedA = angularVelocityToSurfaceSpeed(angularVelocityA, diameterA);
    return surfaceSpeedToAngularVelocity(surfaceSpeedA, diameterB);
  }
}
