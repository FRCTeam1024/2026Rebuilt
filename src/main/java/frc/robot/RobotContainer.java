package frc.robot;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static frc.robot.Constants.*;
import static frc.robot.Constants.ShooterConstants.hubShotRPS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.CommandUtils;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
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
  private final Swerve swerve = new Swerve();

  private final Intake intake = new Intake();
  private final Conveyor conveyor = new Conveyor();
  private final Kicker kicker = new Kicker();
  private final Shooter shooter = new Shooter();
  private final Hood hood = new Hood();
  private final IntakePivot intakePivot = new IntakePivot();
  private final Climber climber = new Climber();

  private final FuelHandler fuelHandler =
      new FuelHandler(intake, conveyor, kicker, shooter, hood, intakePivot);

  private final SendableChooser<Command> autoChooser;

  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve.setDefaultCommand(
        swerve.driveFieldRelativeCmd(
            () -> applyDeadband(-driver.getLeftY(), ControlConstants.stickDeadband),
            () -> applyDeadband(-driver.getLeftX(), ControlConstants.stickDeadband),
            () -> applyDeadband(-driver.getRightX(), ControlConstants.stickDeadband)));

    hood.setDefaultCommand((
      hood.setPositionCommand(() -> 0)
    ));

    NamedCommands.registerCommand(
        "shootFuelFromHub", shooter.velocityCommand(() -> ShooterConstants.hubShotRPS));
    NamedCommands.registerCommand(
        "shootFuelFromSide", shooter.velocityCommand(() -> ShooterConstants.sideShotRPS));
    NamedCommands.registerCommand("climbExtend", climber.autoExtendCommand());
    NamedCommands.registerCommand("climbRetract", climber.autoClimbCommand());
    NamedCommands.registerCommand("extendIntake", fuelHandler.extendIntake());
    NamedCommands.registerCommand(
        "shooterFeed",
        Commands.waitUntil(shooter::atSetpoint).andThen(fuelHandler.feedIntoShooterCommand()));
    NamedCommands.registerCommand("shooterFeedWithHood", Commands.waitUntil(shooter::atSetpoint).andThen(fuelHandler.feedIntoShooterHoodCommand()));
    NamedCommands.registerCommand(
        "retractIntake", intakePivot.setGoalCommand(PivotConstants.stowPosition));


    // Configure the button bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    setupDashboard();
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
    driver.x().onTrue(new InstantCommand(() -> swerve.zeroHeading()));

    // Right trigger or right bumper spins up flywheel
    operator
        .rightTrigger()
        .whileTrue(
            shooter
                .velocityCommand(
                    () -> SmartDashboard.getNumber("Shooter velocity", ShooterConstants.hubShotRPS))
                .finallyDo(shooter::stop));

    operator
        .rightTrigger(0.1)
        .or(operator.rightBumper())
        .and(shooter::atSetpoint)
        .whileTrue(CommandUtils.rumbleController(operator));

    operator.rightBumper().and(shooter::atSetpoint).whileTrue(fuelHandler.feedIntoShooterCommand());


    operator.leftBumper().whileTrue(fuelHandler.vomitCommand());

    operator.leftTrigger(0.1).or(driver.leftTrigger(0.1)).whileTrue(fuelHandler.intakeCommand());

    operator.y().onTrue(intakePivot.setGoalCommand(PivotConstants.intakePosition));
    operator.a().onTrue(intakePivot.setGoalCommand(PivotConstants.stowPosition));
    operator.b().onTrue(intakePivot.setGoalCommand(PivotConstants.insideBumperPosition));

    operator.povUp().or(driver.povUp()).whileTrue(climber.extendCommand());

    operator.povDown().or(driver.povDown()).whileTrue(climber.retractCommand());
    operator.povRight().or(driver.povRight()).onTrue(climber.autoExtendCommand());

    operator.back().whileTrue(intakePivot.currentHome());
    operator.start().whileTrue(hood.currentHome());

    operator.x().whileTrue(new ParallelCommandGroup(shooter.distanceCommand(swerve.getDistanceToHub()).finallyDo(shooter::stop),
                                          hood.distanceCommand(swerve.getDistanceToHub())));
    //operator.x().whileTrue(fuelHandler.tuningModeCommand());

    // shooter.setDefaultCommand(shooter.runIdleCommand(() -> 20));

    // operator.start().onTrue(shooter.sysIdRoutine());

    SmartDashboard.putNumber("Shooter velocity", hubShotRPS);
  }

  public void emergencyStop() {
    shooter.emergencyStop();
  }

  public void setupDashboard() {
    driverTab.add(autoChooser).withPosition(0, 0).withSize(2, 1);
    driverTab.addBoolean("Shooter Ready", () -> shooter.atSetpoint()).withPosition(0, 1);
    Shuffleboard.selectTab("Driver");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
