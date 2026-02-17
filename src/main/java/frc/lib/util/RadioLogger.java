package frc.lib.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;

public class RadioLogger {
  private static final Duration kTimeout = Duration.ofSeconds(1);

  private final StringPublisher statusPublisher =
      NetworkTableInstance.getDefault()
          .getStringTopic("RadioStatus/Status")
          .publishEx("json", "{}");
  private final BooleanPublisher connectedPublisher =
      NetworkTableInstance.getDefault().getBooleanTopic("RadioStatus/Connected").publish();

  private final HttpClient client = HttpClient.newHttpClient();
  private final HttpRequest statusReq;

  public RadioLogger() {
    this(RobotController.getTeamNumber());
  }

  public RadioLogger(int teamNumber) {
    statusReq = HttpRequest.newBuilder(makeStatusURI(teamNumber)).timeout(kTimeout).build();
  }

  public void bind(TimedRobot robot) {
    robot.addPeriodic(this::poll, 2.5, 0.015);
  }

  public void poll() {
    client
        .sendAsync(statusReq, HttpResponse.BodyHandlers.ofString())
        .thenApply(HttpResponse<String>::body)
        .exceptionally(
            (e) -> {
              return "";
            })
        .thenAccept(this::logStatus);
  }

  private void logStatus(String status) {
    statusPublisher.set(status);
    connectedPublisher.set(!status.isEmpty());
  }

  private static URI makeStatusURI(int teamNumber) {
    return URI.create("http://10." + teamNumber / 100 + "." + teamNumber % 100 + ".1/status");
  }
}
