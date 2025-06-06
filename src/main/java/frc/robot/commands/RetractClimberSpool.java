// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RetractClimberSpool extends Command {

  private Climber _climber;

  private double _encoderStartPosition;
  private double _encoderExpectedPosition;
  private double _positionChange = 10;
  
  /** Creates a new ReleaseClimberSpool. */
  public RetractClimberSpool(Climber climber) {
    this._climber = climber;

    addRequirements(_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this._encoderStartPosition = this._climber.getEncoderPosition();
    this._encoderExpectedPosition = this._encoderStartPosition - this._positionChange;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this._climber.setSpeed(.1);
    //this._climber.positionClimber(this._encoderExpectedPosition, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this._climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this._climber.getEncoderPosition() == this._encoderExpectedPosition;
  }
}
