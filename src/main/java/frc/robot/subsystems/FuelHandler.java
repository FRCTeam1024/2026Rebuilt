package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.ShooterParameterCalculator;
import java.util.function.DoubleSupplier;

public class FuelHandler {

  public static final double passingSpeedRPS = 60;
  public static final double passingHoodSetpointRPS = 70;

  private final Intake intake;
  private final Conveyor conveyor;
  private final Kicker kicker;
  private final Shooter shooter;
  private final Hood hood;
  private final IntakePivot intakePivot;

  private final TunableNumber hoodPositionTuner =
      new TunableNumber("FuelHandlerTuning/HoodPosition", 0);
  private final TunableNumber flywheelVelocityTuner =
      new TunableNumber("FuelHandlerTuning/FlywheelVelocity", 0);

  public FuelHandler(
      Intake intake,
      Conveyor conveyor,
      Kicker kicker,
      Shooter shooter,
      Hood hood,
      IntakePivot intakePivot) {
    this.intake = intake;
    this.conveyor = conveyor;
    this.kicker = kicker;
    this.shooter = shooter;
    this.hood = hood;
    this.intakePivot = intakePivot;
  }

  public Command intakeCommand() {
    return Commands.parallel(
        intake.intakeCommand(),
        intakePivot.setGoalCommand(Constants.PivotConstants.intakePosition));
  }

  public Command extendIntake() {
    return intakePivot.setGoalCommand(Constants.PivotConstants.intakePosition);
  }

  public Command feedIntoShooterCommand() {
    return Commands.parallel(
        intake.intakeCommand(), conveyor.feedAutoJamClear(), kicker.feedCommand());
  }

  public Command aimHubCommand(DoubleSupplier distance) {
    ShooterParameterCalculator.ShooterParameters parameters[] =
        new ShooterParameterCalculator.ShooterParameters[1];
    return Commands.parallel(
        Commands.run(
            () -> {
              parameters[0] = ShooterParameterCalculator.calculateHub(distance.getAsDouble());
            }),
        shooter.velocityCommand(() -> parameters[0].shooterVelocity()),
        hood.setPositionCommand(() -> parameters[0].hoodPosition()));
  }

  public Command feedIntoShooterHoodCommand() {
    return Commands.parallel(
        intake.intakeCommand(),
        conveyor.feedCommand(),
        kicker.feedCommand(),
        hood.setPositionCommand(() -> 6));
  }

  public Command vomitCommand() {
    return Commands.parallel(
        intake.ejectCommand(),
        conveyor.ejectCommand(),
        kicker.retractCommand(),
        shooter.velocityCommand(() -> -30),
        intakePivot.setGoalCommand(Constants.PivotConstants.intakePosition));
  }

  public Command passingSetpointCommand(DoubleSupplier distance) {
    ShooterParameterCalculator.ShooterParameters parameters[] =
        new ShooterParameterCalculator.ShooterParameters[1];
    return Commands.parallel(
        Commands.run(
            () -> {
              parameters[0] = ShooterParameterCalculator.calculatePass(distance.getAsDouble());
            }),
        shooter.velocityCommand(() -> parameters[0].shooterVelocity()),
        hood.setPositionCommand(() -> parameters[0].hoodPosition()));
  }

  public Command tuningModeCommand() {
    return Commands.parallel(
        shooter.velocityCommand(flywheelVelocityTuner), hood.setPositionCommand(hoodPositionTuner));
  }
}
