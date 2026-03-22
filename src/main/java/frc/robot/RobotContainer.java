// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Drivebase.IntakeCommand;
import frc.robot.commands.Drivebase.ShooterCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CommandXboxController driverController =
    new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  @SuppressWarnings("unused") //TODO: Remove this
  private final CommandXboxController operatorController =
    new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  public final SwerveSubsystem drivebase = new SwerveSubsystem();
  public final Intake intake = new Intake();
  public final Shooter shooter = new Shooter();

  public final IntakeCommand intakeCommand = new IntakeCommand(intake, () -> operatorController.a().getAsBoolean(), () -> operatorController.b().getAsBoolean(), () -> operatorController.leftBumper().getAsBoolean());
  public final ShooterCommand shooterCommand = new ShooterCommand(shooter, () -> operatorController.rightBumper().getAsBoolean(), () -> operatorController.x().getAsBoolean(), () -> MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), 0.2));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is
   * controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = 
    SwerveInputStream.of(drivebase.getSwerveDrive(),
                         () -> driverController.getLeftY() * -1,
                         () -> driverController.getLeftX())
                     .withControllerRotationAxis(() -> driverController.getRightX())
                     .deadband(OperatorConstants.DEADBAND)
                     .scaleTranslation(0.8)
                     .allianceRelativeControl(true);

  /**
  * Clone's the angular velocity input stream and converts it to a
  * fieldRelative input stream.
  */
  SwerveInputStream driveDirectAngle = 
    driveAngularVelocity.copy().withControllerHeadingAxis(driverController::getRightY,
                                                          driverController::getRightX)
                               .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a
   * robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = 
    driveAngularVelocity.copy().robotRelative(true)
                               .allianceRelativeControl(false);

  /*SwerveInputStream driveAngularVelocityKeyboard = 
    SwerveInputStream.of(drivebase.getSwerveDrive(),
                         () -> -driverController.getLeftY(),
                         () -> -driverController.getLeftX())
                     .withControllerRotationAxis(() -> driverController.getRawAxis(
                                                 2))
                     .deadband(OperatorConstants.DEADBAND)
                     .scaleTranslation(0.8)
                     .allianceRelativeControl(true);*/
  
  // Derive the heading axis with math!
  /*SwerveInputStream driveDirectAngleKeyboard = 
    driveAngularVelocityKeyboard.copy().withControllerHeadingAxis(
      () -> Math.sin(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
      () -> Math.cos(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
                                       .headingWhile(true)
                                       .translationHeadingOffset(true)
                                       .translationHeadingOffset(Rotation2d.fromDegrees(0));*/


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));
    intake.setDefaultCommand(intakeCommand);
    shooter.setDefaultCommand(shooterCommand);

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto();
  }
}
