// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RaiseElevatorCommand;
import frc.robot.subsystems.Elevator;

public class ResetElevator extends SequentialCommandGroup {
  
  public ResetElevator(Elevator elevator) {
    addCommands(
      new RaiseElevatorCommand(elevator, 875),
      new RaiseElevatorCommand(elevator, 90)
    );
  }
}
