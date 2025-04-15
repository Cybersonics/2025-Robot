// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgeaMechanism;
import frc.robot.subsystems.Pneumatics;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgea extends Command {

  private AlgeaMechanism _algea;
  private Timer _commandTimer;

  /** Creates a new IntakeLevelTwoAlgea. */
  public IntakeAlgea(AlgeaMechanism algea) {
    this._algea = algea;
    
    addRequirements(algea);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this._commandTimer = new Timer(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!this._commandTimer.isRunning()) {
      this._commandTimer.start();
    }
    this._algea.intakeAlgea();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this._commandTimer.stop();
    this._commandTimer.reset();
    this._algea.setSpeed(.07);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this._commandTimer.hasElapsed(1.5);
  }
}
