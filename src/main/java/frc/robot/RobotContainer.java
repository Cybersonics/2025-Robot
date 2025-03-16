// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ExtendClimber;
import frc.robot.commands.RetractClimber;
import frc.robot.commands.IntakeAlgea;
import frc.robot.commands.AlgaeMechanismCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.autos.DriveForwardSlow;
import frc.robot.commands.autos.IntakeAlgeaFromReef;
import frc.robot.commands.autos.ScoreAlgaeInBarge;
import frc.robot.commands.autos.ScoreCoralLevelFour;
import frc.robot.commands.autos.ScoreCoralLevelOne;
import frc.robot.commands.autos.ScoreCoralLevelThree;
import frc.robot.commands.autos.ScoreCoralLevelTwo;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.subsystems.AlgeaMechanism;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralMechanism;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;

import frc.robot.subsystems.PigeonGyro;
import frc.robot.subsystems.NavXGyro;

import frc.robot.subsystems.Pneumatics;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

   // Setting up Gyro must be done before Drive as it is used by the Drive
  //public static NavXGyro _gyro = NavXGyro.getInstance();
  public static PigeonGyro _gyro = PigeonGyro.getInstance();

  public static Drive _drive = Drive.getInstance(_gyro);
  
  public static Elevator _elevator = Elevator.getInstance();
  public static AlgeaMechanism _algeaMechanism = AlgeaMechanism.getInstance();
  public static CoralMechanism _coralMechanism = CoralMechanism.getInstance();
  public static Climber _climber = Climber.getInstance();
  public static Pneumatics _pneumatics = Pneumatics.getInstance();

  public static Camera _camera = Camera.getInstance();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DriveController);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OpController);

  // Setup Sendable chooser for picking autonomous program in SmartDashboard
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configured Named commands for pathplanner
    configureNamedCommands();
    
    // Configure Autonomous Options
    autonomousOptions();
    
    // Configure the trigger bindings
    configureBindings();

    // Default Comands always running
    CommandScheduler.getInstance()
    .setDefaultCommand(_drive, new DriveCommand(_drive, driverController, _gyro, _camera));

    CommandScheduler.getInstance()
    .setDefaultCommand(_elevator, new ElevatorCommand(_elevator, operatorController));

    CommandScheduler.getInstance()
    .setDefaultCommand(_algeaMechanism, new AlgaeMechanismCommand(_algeaMechanism, operatorController.rightBumper(), operatorController.leftBumper(), operatorController.pov(180), false));
    
    //CommandScheduler.getInstance()
    //.setDefaultCommand(_climber, new ClimberCommand(_climber, _pneumatics, driverController.pov(0), driverController.pov(180)));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Zero Gyro on Drive B press
    this.driverController.b().onTrue(new InstantCommand(() -> _gyro.zeroGyroHeading()));
    
    this.driverController.pov(0).onTrue(new ClimberCommand(_climber, _pneumatics, driverController.pov(0), driverController.pov(180)));
    this.driverController.pov(180).onTrue(new ClimberCommand(_climber, _pneumatics, driverController.pov(0), driverController.pov(180)));
    // this.driverController.pov(0).onTrue(new ExtendClimber(_pneumatics, _climber));
    // this.driverController.pov(180).onTrue(new RetractClimber(_pneumatics, _climber));

    // Score Coral Levels     
    this.operatorController.pov(90).whileTrue(new ScoreCoralLevelOne(_elevator, _coralMechanism));
    this.operatorController.b().whileTrue(new ScoreCoralLevelTwo(_elevator, _coralMechanism));
    this.operatorController.x().whileTrue(new ScoreCoralLevelThree(_elevator, _coralMechanism));
    this.operatorController.y().whileTrue(new ScoreCoralLevelFour(_elevator, _coralMechanism));

    this.operatorController.rightTrigger().onTrue(new IntakeCoralCommand(_coralMechanism));
    this.operatorController.leftTrigger().onTrue(new ScoreCoralCommand(_coralMechanism));

    this.operatorController.a().onChange(new ConditionalCommand(
      new InstantCommand(() -> _pneumatics.algeaOut()),
      new InstantCommand(() -> _pneumatics.algeaIn()),
      this.operatorController.a()));
  }

  public void configureNamedCommands() {
      NamedCommands.registerCommand("ScoreLevelOne", new ScoreCoralLevelOne(_elevator, _coralMechanism));
      NamedCommands.registerCommand("ScoreLevelTwo", new ScoreCoralLevelTwo(_elevator, _coralMechanism));
      NamedCommands.registerCommand("ScoreLevelThree", new ScoreCoralLevelThree(_elevator, _coralMechanism));
      NamedCommands.registerCommand("ScoreLevelFour", new ScoreCoralLevelFour(_elevator, _coralMechanism));  
      NamedCommands.registerCommand("ScoreAlgeaInBarge", new ScoreAlgaeInBarge(_elevator, _algeaMechanism));

      NamedCommands.registerCommand("IntakeCoral", new IntakeCoralCommand(_coralMechanism));
      NamedCommands.registerCommand("IntakeAlgeaFromReef", new IntakeAlgeaFromReef(_elevator, _algeaMechanism, _pneumatics));
      NamedCommands.registerCommand("DriveSlowForward", new DriveForwardSlow(_drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  /**
   * Use this to set Autonomous options for selection in Smart Dashboard
   */
  private void autonomousOptions() {
    // Example adding Autonomous option to chooser
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

  }
}
