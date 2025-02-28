// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class RaiseElevatorCommand extends Command {

  private Elevator _elevatorSubsytem;
  private double _levelHeight;
  private double _upSpeed;
  private double _downSpeed;

  public RaiseElevatorCommand(Elevator elevator, double levelHeight) {
    this._elevatorSubsytem = elevator;
    this._levelHeight = levelHeight;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _upSpeed = .7;
    _downSpeed = .5;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentHeight = this._elevatorSubsytem.getAlgeaHeight();
    System.out.println("Moving from " + currentHeight + " to " + _levelHeight);
    if (currentHeight > _levelHeight) {
      System.out.println("Lowering Elevator");
      // 70 is bottom stop before then let it settle on own with gravity.
      if (this._elevatorSubsytem.getAlgeaHeight() > 100) {
        // Don't jam into bottom slow
        if (this._elevatorSubsytem.getAlgeaHeight() < 1200) {
          this._elevatorSubsytem.setSpeed(-_downSpeed);
        } else {
          this._elevatorSubsytem.setSpeed(-_downSpeed);
        }
      } else {
        this._elevatorSubsytem.stop();
      }
    } else if (currentHeight < _levelHeight) {
      System.out.println("Raising Elevator");
      this._elevatorSubsytem.setSpeed(_upSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this._elevatorSubsytem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this._elevatorSubsytem.getAlgeaHeight() >= _levelHeight - 35
        && this._elevatorSubsytem.getAlgeaHeight() <= _levelHeight + 35;
  }
}
