// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaeMechanismCommand;
import frc.robot.commands.RaiseElevatorCommand;
import frc.robot.subsystems.AlgeaMechanism;
import frc.robot.subsystems.Elevator;

public class ScoreAlgaeInBarge extends SequentialCommandGroup {

  public ScoreAlgaeInBarge(Elevator elevator, AlgeaMechanism algea) {
    addCommands(
      new RaiseElevatorCommand(elevator, 3675),
      new AlgaeMechanismCommand(algea, () -> false, () -> true, true),
      new RaiseElevatorCommand(elevator, 70)
    );
  }
}
