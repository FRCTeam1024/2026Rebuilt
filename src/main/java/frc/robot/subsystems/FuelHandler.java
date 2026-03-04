package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class FuelHandler {

  private final Intake intake;
  private final Conveyor conveyor;
  private final Kicker kicker;
  private final Shooter shooter;
  private final Hood hood;
  private final IntakePivot intakePivot;

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
        intake.intakeCommand(), conveyor.oscillateCommand(), kicker.feedCommand());
  }

  public Command vomitCommand() {
    return Commands.parallel(
        intake.ejectCommand(),
        conveyor.ejectCommand(),
        kicker.retractCommand(),
        shooter.velocityCommand(() -> -30),
        intakePivot.setGoalCommand(Constants.PivotConstants.intakePosition));
  }
}
