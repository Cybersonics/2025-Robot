// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Kelvin;

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
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentHeight = this._elevatorSubsytem.getAlgeaHeight();
    System.out.println("Moving from " + currentHeight +" to " +_levelHeight);
    if(currentHeight > _levelHeight) {
      System.out.println("Lowering Elevator");
      this._elevatorSubsytem.setSpeed(-.2);
    } else if (currentHeight < _levelHeight) {
      System.out.println("Raising Elevator");
      this._elevatorSubsytem.setSpeed(.2);
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
    return this._elevatorSubsytem.getAlgeaHeight() >= _levelHeight - 10
      && this._elevatorSubsytem.getAlgeaHeight() <= _levelHeight + 10;
  }
}
