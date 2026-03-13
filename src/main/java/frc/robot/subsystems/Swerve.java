package frc.robot.subsystems;

import static frc.robot.Constants.SwerveConstants;
import static frc.robot.Constants.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AprilTagVision;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.FieldPoses;
import frc.robot.SwerveModule;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import monologue.Logged;

public class Swerve extends SubsystemBase implements Logged {
  private SwerveDrivePoseEstimator poseEstimator;
  private SwerveModule[] swerveMods;
  private ProfiledPIDController headingController;
  private SimpleMotorFeedforward headingFeedforward;
  private Canandgyro gyro = new Canandgyro(gyroID);
  private AprilTagVision vision = new AprilTagVision();

  private double hubDistance;

  public Swerve() {
    gyro.setYaw(0);

    gyro.resetFactoryDefaults(0.02);

    swerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
          new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
          new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
          new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            SwerveConstants.initialPose,
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(45)));

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      // TODO: handle gracefully
      throw new RuntimeException("Failed to load config");
    }
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getRobotRelativeChassisSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(Constants.AutoConstants.kPXController),
            new PIDConstants(Constants.AutoConstants.kPThetaController)),
        config,
        Swerve::shouldFlipPath,
        this // Reference to this subsystem to set requirements
        );

    headingController =
        new ProfiledPIDController(
            Constants.SwerveConstants.headingkP,
            Constants.SwerveConstants.headingkI,
            Constants.SwerveConstants.headingkD,
            new TrapezoidProfile.Constraints(
                Constants.SwerveConstants.maxAngularVelocity, Constants.SwerveConstants.maxAngularAcceleration));

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    headingFeedforward =
        new SimpleMotorFeedforward(
            Constants.SwerveConstants.headingkS, Constants.SwerveConstants.headingkV, Constants.SwerveConstants.headingkA);
  }

  private static boolean shouldFlipPath() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

    ChassisSpeeds robotRelativeChassisSpeeds;
    if (fieldRelative) {
      robotRelativeChassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(), translation.getY(), rotation, getHeading());
    } else {
      robotRelativeChassisSpeeds =
          new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    SwerveModuleState[] swerveModuleStates =
        SwerveConstants.swerveKinematics.toSwerveModuleStates(robotRelativeChassisSpeeds);

    setModuleStates(swerveModuleStates, isOpenLoop);
  }
public void alignmentDrive(Pose2d theTarget) {
    Pose2d current = getPose();
    Transform2d transform = theTarget.minus(current);
    double xError = transform.getX();
    double yError = transform.getY();
    double rotError = transform.getRotation().getRadians();
    double pTrans = 1;
    double pRot = 1;
    double sGain = 0.01;

    log("xError", xError);
    log("yError", yError);
    log("rotError", rotError);

    double xDrive = MathUtil.clamp(sGain * Math.signum(xError) + xError * pTrans, -1, 1);
    double yDrive = MathUtil.clamp(sGain * Math.signum(yError) + yError * pTrans, -1, 1);
    double rotDrive = MathUtil.clamp(sGain * Math.signum(rotError) + rotError * pRot, -1, 1);

    if (Math.sqrt(xError*xError + yError*yError) < 0.7 && Math.abs(rotError) < 1 ) {
      drive(
        new Translation2d(xDrive, yDrive).times(Constants.SwerveConstants.maxSpeed),
        rotDrive * Constants.SwerveConstants.maxAngularVelocity,
        false,
        true);
    }
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(swerveModuleStates, false);
  }

  public void teleopDriveCommand() {

  }

  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : swerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : swerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d heading) {
    poseEstimator.resetPosition(
        getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroHeading() {
    poseEstimator.resetPosition(
        getGyroYaw(),
        getModulePositions(),
        new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public Rotation2d getGyroYaw() {
    return gyro.getRotation2d();
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : swerveMods) {
      mod.resetToAbsolute();
    }
  }

  public Command driveFieldRelativeCmd(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, BooleanSupplier aim) {
    return run(
        () -> {
          double translationVal = x.getAsDouble();
          double strafeVal = y.getAsDouble();
          double rotationVal = omega.getAsDouble();

          if (shouldFlipPath()) {
              translationVal = translationVal * -1;
              strafeVal = strafeVal * -1;
          }

          if (aim.getAsBoolean()) {
              Rotation2d goalHeading = getPose().getTranslation().minus(FieldPoses.getHubCenter()).getAngle();
              if (Math.abs(goalHeading.minus(getHeading()).getRadians()) > Constants.SwerveConstants.headingGoalRange) {
                  rotationVal = headingController.calculate(
                    MathUtil.angleModulus(getHeading().getRadians()),
                    MathUtil.angleModulus(goalHeading.getRadians()));
                  rotationVal += headingFeedforward.calculate(headingController.getSetpoint().velocity);
              }
              else {
                rotationVal = 0;
              }
          }
          drive(
              new Translation2d(translationVal, strafeVal)
                  .times(Constants.SwerveConstants.maxSpeed),
              rotationVal * Constants.SwerveConstants.maxAngularVelocity,
              true,
              true);
        });
  }

  public Command autoClimbAdjust(boolean goLeft) {
    Pose2d target;
    if (shouldFlipPath()) {
      if (goLeft) {
        target = Constants.ClimberConstants.leftRedClimbPose;
      }
      else {
        target = Constants.ClimberConstants.rightRedClimbPose;
      }
    }
    else {
      if (goLeft) {
        target = Constants.ClimberConstants.leftBlueClimbPose;
      }
      else {
        target = Constants.ClimberConstants.rightBlueClimbPose;
      }
    }
    return run(() -> alignmentDrive(target));
  }

  public DoubleSupplier getDistanceToHub() {
    return () -> hubDistance;
  }

  @Override
  public void periodic() {
    for (SwerveModule mod : swerveMods) {
      mod.refresh();
      log("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
      log("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      log("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
    poseEstimator.update(getGyroYaw(), getModulePositions());
    vision.processVisionUpdates(
        (estimatedPose) -> {
          if (Constants.aprilTagsEnabled) {
            poseEstimator.addVisionMeasurement(
                estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
          }
        },
        getPose());

    log("Pose", poseEstimator.getEstimatedPosition());
    var hubCenter =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Blue
            ? FieldPoses.blueHubCenter
            : FieldPoses.redHubCenter;
    hubDistance = getPose().getTranslation().getDistance((hubCenter));
    log("Distance to center of hub", hubDistance);
  }
}
