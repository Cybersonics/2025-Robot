// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class RaiseElevatorCommand extends Command {

  private Elevator _elevatorSubsytem;
  private double _levelHeight;


  public RaiseElevatorCommand(Elevator elevator, double levelHeight) {
    this._elevatorSubsytem = elevator;
    this._levelHeight = levelHeight;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this._elevatorSubsytem.setElevatorHeight(_levelHeight);  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _elevatorSubsytem.elevatorAtSetpoint() && _elevatorSubsytem.elevatorAtGoal();
  }
}
