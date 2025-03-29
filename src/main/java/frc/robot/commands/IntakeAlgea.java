// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeaMechanism;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgea extends Command {

  private AlgeaMechanism _algea;

  /** Creates a new IntakeLevelTwoAlgea. */
  public IntakeAlgea(AlgeaMechanism algea) {
    this._algea = algea;
    
    addRequirements(algea);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this._algea.ejectAlgea();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this._algea.setSpeed(-.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this._algea.HasAmpCurrentSpiked();
  }
}
