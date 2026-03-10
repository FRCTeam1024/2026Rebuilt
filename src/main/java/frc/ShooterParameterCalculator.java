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
    map.put(1.706, new ShooterParameters(10,42));// Side auto shot
    map.put(1.962, new ShooterParameters(20,45));//1 robot away from hub
    map.put(2.267, new ShooterParameters(22,45));
    map.put(2.572, new ShooterParameters(24,47));
    map.put(3.149, new ShooterParameters(25,50));//front of tower shop test
    map.put(3.900, new ShooterParameters(35,54));
    map.put(5.245, new ShooterParameters(67,57)); //Corner 
    //map.put(4.096, new ShooterParameters(70,57));//test was limited by ceiling height
  
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
