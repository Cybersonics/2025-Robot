// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeaMechanism;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pneumatics;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberCommand extends Command {

  private Climber _climber;
  private Pneumatics _pneumatics;
  private BooleanSupplier _extend;
  private BooleanSupplier _retract;
  
  /** Creates a new ClimberCommand. */
  public ClimberCommand(Climber climber, Pneumatics pnematics, BooleanSupplier extend, BooleanSupplier retract) {
    // Use addRequirements() here to declare subsystem dependencies.
    this._climber = climber;
    this._pneumatics = pnematics;
    this._extend = extend;
    this._retract = retract;

    addRequirements(climber, pnematics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this._extend.getAsBoolean()) {
      this._climber.setSpeed(-.6);
      this._pneumatics.climberOut();

    } else if (this._retract.getAsBoolean()) {
      this._climber.setSpeed(.75);
      this._pneumatics.climberIn();

    } else {
      this._climber.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this._climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
