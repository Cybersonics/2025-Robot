// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ScoreTroughCoralCommand;
import frc.robot.commands.RaiseElevatorCommand;
import frc.robot.subsystems.CoralMechanism;
import frc.robot.subsystems.Elevator;

public class ScoreCoralLevelOne extends SequentialCommandGroup {
  
  public ScoreCoralLevelOne(Elevator elevator, CoralMechanism coral) {
    addCommands(
      new RaiseElevatorCommand(elevator, 750), //825   680  700
      new ScoreTroughCoralCommand(coral)
    );
  }
}
