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
    hubMap.put(1.268, new ShooterParameters(0, 38)); // hub shot
    hubMap.put(1.706, new ShooterParameters(3, 42)); // Side auto shot
    hubMap.put(1.962, new ShooterParameters(7, 42)); // 1 robot away from hub
    hubMap.put(2.267, new ShooterParameters(7, 44));
    hubMap.put(2.572, new ShooterParameters(7, 46));
    hubMap.put(3.13, new ShooterParameters(10, 50)); // front of tower shop test
    hubMap.put(3.45, new ShooterParameters(12, 52));
    // hubMap.put(3.45, new ShooterParameters(11, 54)); This was ~ok, shots were really high with
    // less consistency
    hubMap.put(3.76, new ShooterParameters(16, 53)); // Corner
    hubMap.put(3.900, new ShooterParameters(19, 54));
    hubMap.put(4.5, new ShooterParameters(27, 56));
    hubMap.put(5.21, new ShooterParameters(32.5, 60)); // Corner

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
