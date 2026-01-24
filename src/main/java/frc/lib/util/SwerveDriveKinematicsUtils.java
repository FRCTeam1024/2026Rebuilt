package frc.lib.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveDriveKinematicsUtils {
  /**
   * Create a SwerveDriveKinematics object for a rectangular chassis from the given wheelbase
   * dimensions. The kinematics origin is in the center of the four wheels.
   *
   * @param wheelBaseX The distance between the center of the from front to back
   * @param wheelBaseY The distance between the center of the wheels from left to right
   * @return A new SwerveDriveKinematics object
   */
  public static SwerveDriveKinematics fromWheelBases(double wheelBaseX, double wheelBaseY) {
    return new SwerveDriveKinematics(
        new Translation2d(wheelBaseX / 2.0, wheelBaseY / 2.0),
        new Translation2d(wheelBaseX / 2.0, -wheelBaseY / 2.0),
        new Translation2d(-wheelBaseX / 2.0, wheelBaseY / 2.0),
        new Translation2d(-wheelBaseX / 2.0, -wheelBaseY / 2.0));
  }
}
