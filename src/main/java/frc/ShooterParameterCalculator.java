package frc;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShooterParameterCalculator {

  public static InterpolatingTreeMap<Double, ShooterParameters> map =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterParameters::interpolate);

  static {
    map.put(0.5, new ShooterParameters(0, 42.0)); 
    map.put(1.268, new ShooterParameters(0, 42.0)); //hub shot
    map.put(1.706, new ShooterParameters(20,53));// Side auto shot
  }

  public record ShooterParameters(double hoodPosition, double shooterVelocity) {
    public static ShooterParameters interpolate(
        ShooterParameters start, ShooterParameters end, double t) {
      return new ShooterParameters(
          MathUtil.interpolate(start.hoodPosition, end.hoodPosition, t),
          MathUtil.interpolate(start.shooterVelocity, end.shooterVelocity, t));
    }
  }

  public static ShooterParameters calculate(double distance) {
    return map.get(distance);
  }
}
