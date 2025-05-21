// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.commands.RaiseElevatorCommand;
import frc.robot.subsystems.CoralMechanism;
import frc.robot.subsystems.Elevator;

public class ScoreCoralLevelThree extends SequentialCommandGroup {
  
  public ScoreCoralLevelThree(Elevator elevator, CoralMechanism coral) {
    addCommands(
      new RaiseElevatorCommand(elevator, 2025), //original 1950, 2050
      new ScoreCoralCommand(coral)
    );
  }
}
