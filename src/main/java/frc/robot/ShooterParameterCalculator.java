package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShooterParameterCalculator {

  public static InterpolatingTreeMap<Double, ShooterParameters> hubMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterParameters::interpolate);

  public static InterpolatingTreeMap<Double, ShooterParameters> passMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterParameters::interpolate);

  static {
    hubMap.put(0.5, new ShooterParameters(0, 42.0));
    hubMap.put(1.268, new ShooterParameters(0, 42.0)); // hub shot
    hubMap.put(1.706, new ShooterParameters(10, 42)); // Side auto shot
    hubMap.put(1.962, new ShooterParameters(20, 45)); // 1 robot away from hub
    hubMap.put(2.267, new ShooterParameters(22, 45));
    hubMap.put(2.572, new ShooterParameters(24, 47));
    hubMap.put(3.149, new ShooterParameters(25, 50)); // front of tower shop test
    hubMap.put(3.900, new ShooterParameters(35, 54));
    hubMap.put(5.245, new ShooterParameters(67, 57)); // Corner
    // map.put(4.096, new ShooterParameters(70,57));//test was limited by ceiling height
  }

  static {
    passMap.put(5.0, new ShooterParameters(41, 55));
    passMap.put(5.8, new ShooterParameters(45, 55));
    passMap.put(6.65, new ShooterParameters(53, 58));
    passMap.put(7.75, new ShooterParameters(57, 65));
    passMap.put(8.8, new ShooterParameters(68, 65));
  }

  public record ShooterParameters(double hoodPosition, double shooterVelocity) {
    public static ShooterParameters interpolate(
        ShooterParameters start, ShooterParameters end, double t) {
      return new ShooterParameters(
          MathUtil.interpolate(start.hoodPosition, end.hoodPosition, t),
          MathUtil.interpolate(start.shooterVelocity, end.shooterVelocity, t));
    }
  }

  public static ShooterParameters calculateHub(double distance) {
    return hubMap.get(distance);
  }

  public static ShooterParameters calculatePass(double distance) {
    return passMap.get(distance);
  }
}
