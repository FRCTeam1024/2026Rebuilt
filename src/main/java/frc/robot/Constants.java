package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveDriveKinematicsUtils;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static final AprilTagFieldLayout kAndyMarkField =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  public static boolean aprilTagsEnabled = false;

  public static final class ControlConstants {
    public static final int driverPort = 0;
    public static final double stickDeadband = 0.04;

    public static final int operatorPort = 1;
  }

  public static final class SwerveConstants {
    public static final int gyroID = 0;

    /** Drive motor rotations per rotation of azimuth */
    public static final double azimuthCouplingRatio =
        54.0 / 14.0; // TODO: This must be set for specific module

    /* Drivetrain Constants */
    public static final double trackWidth =
        Units.inchesToMeters(21.626); // TODO: This must be tuned to specific robot
    public static final double wheelBase =
        Units.inchesToMeters(21.626); // TODO: This must be tuned to specific robot
    public static final double wheelCircumference = Units.inchesToMeters(4 * Math.PI);

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        SwerveDriveKinematicsUtils.fromWheelBases(wheelBase, trackWidth);

    /* Module Gear Ratios */
    public static final double driveGearRatio = 6.03;
    public static final double angleGearRatio = 287.0 / 11.0;

    /* Motor Inverts */

    /**
     * Direction the angle motor needs to go for the azimuth to move couterclockwise when viewed
     * from the top
     */
    public static final InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;

    /** Direction the drive motor needs to go to drive the wheel "forwards" */
    public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;

    /**
     * Angle Encoder Invert Encoder should read positive velocity when azimuth is rotated
     * counterclockwise when viewed from the top
     */
    public static final SensorDirectionValue cancoderInvert =
        SensorDirectionValue.CounterClockwise_Positive;

    /* Swerve Current Limiting */
    public static final int angleSupplyCurrentLimit = 25;
    public static final int angleSupplyCurrentLower = 40;
    public static final double angleSupplyCurrentLowerTime = 0.1;
    public static final boolean angleEnableSupplyCurrentLimit = true;
    public static final double angleStatorCurrentLimit = 80;
    public static final boolean angleEnableStatorCurrentLimit = true;

    public static final int driveSupplyCurrentLimit = 35;
    public static final int driveSupplyCurrentLower = 60;
    public static final double driveSupplyCurrentLowerTime = 0.1;
    public static final boolean driveEnableSupplyCurrentLimit = true;
    public static final double driveStatorCurrentLimit = 60;
    public static final boolean driveEnableStatorCurrentLimit = true;

    /* These values are used by the drive motor controller to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /*
     * Angle Motor PID Values
     * Feedback unit is rotations of azimuth
     * Output unit is voltage
     */
    public static final double angleKP = 100.0;
    public static final double angleKI = 0;
    public static final double angleKD = 0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
    public static final double driveKV = 1.51;
    public static final double driveKA = 0.27;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot

    /** Radians per Second */
    public static final double maxAngularVelocity =
        10.0; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(44.385);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(69.609);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 23;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(48.779);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 31;
      public static final int angleMotorID = 32;
      public static final int canCoderID = 33;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(132.012);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /** The pose used to initialize the pose estimator */
    public static final Pose2d initialPose = Pose2d.kZero;
  }

  public static final class IntakeConstants {
    public static final int motorID = 40;
    public static final double maxOutputVoltage = 10;
  }

  public static final class ConveyorConstants {
    public static final int motorID = 41;
    public static final double maxOutputVoltage = 10;

    public static final double oscillateForwardSpeed = 0.3;
    public static final double oscillateReverseSpeed = -0.1;

    public static final double oscillateForwardTime = 0.3;
    public static final double oscillateOffTime1 = 0.1;
    public static final double oscillateReverseTime = 0.1;
    public static final double oscillateOffTime2 = 0.05;
  }

  public static final class KickerConstants {
    public static final int upperShaftMotorID = 42;
    public static final int lowerShaftMotorID = 43;

    public static final double maxOutputVoltage = 10;
  }

  public static final class ShooterConstants {
    public static final int leftID = 44;
    public static final int rightID = 45;

    public static final double velocityToleranceRPS = 2.5;

    public static final double hubShotRPS = 42.0;

    /** Maximum rate of change for velocity setpoint in RPS per second */
    public static final double velocitySlewRateRPSPerSecond = 20.0;
  }

  public static final class HoodConstants {
    public static final int motorID = 46;

    public static final double motorToHoodRatio =
        3.0 * (48.0 / 24.0); // 3:1 MP, 24T:48T belt reduction

    /* Hood position limits in degrees */
    public static final double minPosition = 0.0; // TODO: retune
    public static final double maxPosition = 90.0;

    public static final double kP = 6.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.1;
    public static final double kA = 0.0;
    public static final double kG = 0.0;
    public static final double cruiseVelocity = 100.0;
    public static final double acceleration = 1500.0;

    public static final double statorCurrentLimit = 40.0;

    public static final double homePosition = 0; // Degrees from horizontal
  }

  public static final class PivotConstants {
    public static final int motorID = 48;

    // Ratio from motor to crank arm
    // 2 3:1 MP stages, 20T:50T spur, 20T:64T spur
    public static final double motorToPivotRatio = 3.0 * 3.0 * (50.0 / 20.0) * (64.0 / 20.0);

    public static final double kP = 150;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.19; // manually tuned
    // 12V / (free speed @ 12v / gear ratio)
    public static final double kV = 12.0 / (6000.0 / 60.0 / motorToPivotRatio);
    public static final double kA = 0;
    public static final double kG = 0;
    public static final double cruiseVelocity = 0.5; // 1;
    public static final double acceleration = 10; // 15;

    public static final double homeVoltage = -1.0;

    // All positions are of the crank arm in rotations. Currently 0 is all the way back.
    public static final double reverseLimit = 0.0;
    public static final double forwardLimit = 0.25;
    public static final double homePosition = 0;
    public static final double intakePosition = 0.24; // No kicker bar
    public static final double stowPosition = 0;
  }

  public static final class ClimberConstants {
    public static final int motorID = 50;

    public static final double extendLimit = -170; // TODO: tune
    public static final double retractLimit = 0;

    public static final double maxOutputVoltage = 12;

    public static final double retractOutput = 1;
    public static final double extendOutput = -1;

    public static final int limitSwitchPin = 1;
  }

  public static final
  class AutoConstants { // TODO: The below constants are used in the example auto, and must be tuned
    // to specific robot
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
