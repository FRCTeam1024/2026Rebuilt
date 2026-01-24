package frc.robot;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import monologue.Logged;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Logged {
  /* Controllers */
  private final CommandXboxController driver =
      new CommandXboxController(ControlConstants.driverPort);

  private final CommandXboxController operator =
      new CommandXboxController(ControlConstants.operatorPort);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

  private final Intake intake = new Intake();
  private final Conveyor conveyor = new Conveyor();
  private final Kicker kicker = new Kicker();
  private final Shooter shooter = new Shooter();
  private final Hood hood = new Hood();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        s_Swerve.driveFieldRelativeCmd(
            () -> applyDeadband(-driver.getLeftY(), ControlConstants.stickDeadband),
            () -> applyDeadband(-driver.getLeftX(), ControlConstants.stickDeadband),
            () -> applyDeadband(-driver.getRightX(), ControlConstants.stickDeadband)));

    // Configure the button bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /* Driver Buttons */
    driver.x().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

    operator
        .leftTrigger()
        .whileTrue(
            shooter
                .spinUpCommand()
                .withTimeout(1)
                .andThen(
                    Commands.parallel(
                        intake.intakeCommand(),
                        conveyor.feedCommand(),
                        kicker.feedCommand(),
                        shooter.spinUpCommand())));

    operator
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                intake.ejectCommand(),
                conveyor.ejectCommand(),
                kicker.retractCommand(),
                shooter.spinUpCommand(() -> -4)));

    operator.rightTrigger().whileTrue(intake.intakeCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
