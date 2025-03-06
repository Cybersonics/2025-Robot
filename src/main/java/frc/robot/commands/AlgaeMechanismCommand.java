// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeaMechanism;

public class AlgaeMechanismCommand extends Command {

  private AlgeaMechanism _algea;
  private boolean _isAuto;
  private BooleanSupplier _shouldIntake;
  private BooleanSupplier _shouldEject;
  private BooleanSupplier _shouldSlowPull;

  private Timer _timer;

  public AlgaeMechanismCommand(AlgeaMechanism algea, BooleanSupplier shouldIntake, BooleanSupplier shouldEject,
      BooleanSupplier shouldSlowPull, boolean isAuto) {
    this._algea = algea;
    this._isAuto = isAuto;
    this._shouldIntake = shouldIntake;
    this._shouldEject = shouldEject;
    this._shouldSlowPull = shouldSlowPull;

    addRequirements(algea);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this._timer = new Timer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this._timer.start();
    if (this._shouldIntake.getAsBoolean()) {
      this._algea.intakeAlgea();
    } else if (this._shouldEject.getAsBoolean()) {
      this._algea.ejectAlgea();
    } else if (this._shouldSlowPull.getAsBoolean()) {
      this._algea.setSpeed(-.07);
    } else {
      this._algea.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this._timer.stop();
    this._timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (this._isAuto) {
      return this._timer.hasElapsed(.5);
    } else {
      return false;
    }
  }
}
