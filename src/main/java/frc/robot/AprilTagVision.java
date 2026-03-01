package frc.robot;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import monologue.Logged;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagVision implements Logged {
  private List<VisionCamera> cameras = new ArrayList<>();

  public AprilTagVision() {
    addCamera("Left Camera", leftCameraName, robotToLeftCamera);
    addCamera("Right Camera", rightCameraName, robotToRightCamera);
  }

  public void addCamera(String name, String photonName, Transform3d robotToCamera) {
    var camera = new PhotonCamera(photonName);
    var estimator = new PhotonPoseEstimator(Constants.kAndyMarkField, robotToCamera);
    cameras.add(new VisionCamera(name, camera, estimator));
  }

  public void processVisionUpdates(Consumer<EstimatedRobotPose> poseConsumer, Pose2d curPose) {
    for (var camera : cameras) {
      var results = camera.camera().getAllUnreadResults();
      log(camera.name() + " Num Targets", results.size());
      if (!results.isEmpty()) {
        var result = results.get(results.size() - 1);
        var estimatedPose = camera.estimator().estimateCoprocMultiTagPose(result);
        if (estimatedPose.isPresent()) {
          log(camera.name() + "EstimatedPoseRaw", estimatedPose.get().estimatedPose);
          if (shouldAcceptUpdate(result, estimatedPose.get(), curPose)) {
            poseConsumer.accept(estimatedPose.get());
            log(camera.name() + "EstimatedPoseAccepted", estimatedPose.get().estimatedPose);
          }
        }
      }
      log(camera.name() + " Connected", camera.camera().isConnected());
    }
  }

  private static final Set<Integer> allowedTags = Set.of();

  private static final boolean limitTags = false;

  public boolean shouldAcceptUpdate(
      PhotonPipelineResult result, EstimatedRobotPose estimatedRobotPose, Pose2d curPose) {

    if (limitTags) {
      if (!allowedTags.contains(estimatedRobotPose.targetsUsed.get(0).fiducialId)) {
        return false;
      }
    }

    // Reject if off field
    if (!translationWithinField(estimatedRobotPose.estimatedPose.toPose2d().getTranslation())) {
      return false;
    }

    // Reject if > 6" above or below the ground
    if (Math.abs(estimatedRobotPose.estimatedPose.getZ()) > Units.inchesToMeters(6)) {
      return false;
    }

    // Reject if high ambiguity
    if (estimatedRobotPose.targetsUsed.get(0).poseAmbiguity > 0.5) {
      return false;
    }

    // Always accept if we have a multitag result and the size is large enough
    if (result.getMultiTagResult().isPresent()
        && estimatedRobotPose.targetsUsed.get(0).getArea() > 0.05) {
      return true;
    }

    var apriltagPose =
        Constants.kAndyMarkField.getTagPose(estimatedRobotPose.targetsUsed.get(0).getFiducialId());
    if (apriltagPose.isEmpty()) {
      return false;
    }

    // Reject if tag is too far away
    var distanceToTag =
        curPose.getTranslation().getDistance(apriltagPose.get().getTranslation().toTranslation2d());
    log("distance to tag", distanceToTag);

    if (distanceToTag > 7) {
      return false;
    }

    return true;
  }

  private boolean translationWithinBounds(
      Translation2d value, Translation2d min, Translation2d max) {
    return (value.getX() > min.getX() && value.getX() < max.getX())
        && (value.getY() > min.getY() && value.getY() < max.getY());
  }

  private boolean translationWithinField(Translation2d val) {
    return translationWithinBounds(
        val,
        new Translation2d(),
        new Translation2d(
            Constants.kAndyMarkField.getFieldLength(), Constants.kAndyMarkField.getFieldWidth()));
  }

  public static record VisionCamera(
      String name, PhotonCamera camera, PhotonPoseEstimator estimator) {}
}
