// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pneumatics;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RetractAlgeaIntake extends Command {

  private Pneumatics _pneumatics;
  private Timer _timer;
  private boolean _isFinished = false;

  /** Creates a new ExtendAlgeaIntake. */
  public RetractAlgeaIntake(Pneumatics pneumatics) {
    this._pneumatics = pneumatics;

    addRequirements(_pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this._isFinished = false;
    this._timer = new Timer();
    this._timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this._pneumatics.algeaIn();
    if(this._timer.hasElapsed(.5)) {
    System.out.println("Algea In Timer passed");
      this._isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this._isFinished;
  }
}
