// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ExtendAlgeaIntake;
import frc.robot.commands.IntakeAlgea;
import frc.robot.commands.RaiseElevatorCommand;
import frc.robot.commands.RetractAlgeaIntake;
import frc.robot.subsystems.AlgeaMechanism;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pneumatics;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgeaFromReef extends SequentialCommandGroup {
 
  public IntakeAlgeaFromReef(Elevator elevator, AlgeaMechanism algea, Pneumatics pneumatics) {
    addCommands(
      new RaiseElevatorCommand(elevator, 1250),
      new ExtendAlgeaIntake(pneumatics),
      new IntakeAlgea(algea),
      new RetractAlgeaIntake(pneumatics)
    );
  }
}
