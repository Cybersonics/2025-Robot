// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.commands.RaiseElevatorCommand;
import frc.robot.subsystems.CoralMechanism;
import frc.robot.subsystems.Elevator;

public class ScoreCoralLevelTwo extends SequentialCommandGroup {
  
  public ScoreCoralLevelTwo(Elevator elevator, CoralMechanism coral) {
    addCommands(
      new RaiseElevatorCommand(elevator, 1200),
      new ScoreCoralCommand(coral),
      new RaiseElevatorCommand(elevator, 70)
    );
  }
}
