// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pneumatics;

public class RetractClimber extends SequentialCommandGroup {
  /** Creates a new RetractClimber. */
  public RetractClimber(Pneumatics pneumatics, Climber climber) {
    addCommands(
      new InstantCommand(() -> pneumatics.climberIn()),
      new RetractClimberSpool(climber)
    );
  }
}