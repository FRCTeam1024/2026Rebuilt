package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldPoses {
  public static final Translation2d blueHubCenter = new Translation2d(4.620419, 4.034631);
  public static final Translation2d redHubCenter = new Translation2d(11.910219, 4.034631);

  public static final double bluePassingLineX = 1.675;
  public static final double redPassingLineX =
      Constants.kAndyMarkField.getFieldLength() - bluePassingLineX;

  public static Translation2d getHubCenter() {
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
    return isRed ? redHubCenter : blueHubCenter;
  }

  public static double getPassingLineX() {
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
    return isRed ? redPassingLineX : bluePassingLineX;
  }
}
