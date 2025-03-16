// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class RaiseElevatorCommand extends Command {

  private Elevator _elevatorSubsytem;
  private double _levelHeight;
  //private double _upSpeed;
  //private double _downSpeed;

  //private PIDController _elevatorPIDController;
  //private ElevatorFeedforward _feedforward;

  public RaiseElevatorCommand(Elevator elevator, double levelHeight) {
    this._elevatorSubsytem = elevator;
    this._levelHeight = levelHeight;

    //double p = .012; //0.0093
    //double i = 0;
    //double d = 0.0005;//0.0002

    //double kS = 0.001;//0.01
    //double kG = 0.01;//0.41
    //double kV = 0.0002;//0.002
    //_feedforward = new ElevatorFeedforward(kS, kG, kV);

    //this._elevatorPIDController = new PIDController(p, i, d);
    //this._elevatorPIDController.setTolerance(25);

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //_upSpeed = .7;
    //_downSpeed = .5;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this._elevatorSubsytem.setElevatorPosition(_levelHeight);

    //double currentHeight = this._elevatorSubsytem.getAlgeaHeight();
    //System.out.println("Moving from " + currentHeight + " to " + _levelHeight);

    //double calcVoltage = this._elevatorPIDController.calculate(currentHeight, _levelHeight);
    //SmartDashboard.putNumber("Current Elevator Voltage", calcVoltage);

    //double feedForward = _feedforward.calculate(this._elevatorSubsytem.getEncoderVelocity());
    //SmartDashboard.putNumber("Feed Forward Voltage", feedForward);

    //this._elevatorSubsytem.setVoltage(calcVoltage + feedForward);

    // if (currentHeight > _levelHeight) {
    // System.out.println("Lowering Elevator");
    // // 70 is bottom stop before then let it settle on own with gravity.
    // if (this._elevatorSubsytem.getAlgeaHeight() > 100) {
    // // Don't jam into bottom slow
    // if (this._elevatorSubsytem.getAlgeaHeight() < 1200) {
    // this._elevatorSubsytem.setSpeed(-_downSpeed);
    // } else {
    // this._elevatorSubsytem.setSpeed(-_downSpeed);
    // }
    // } else {
    // this._elevatorSubsytem.stop();
    // }
    // } else if (currentHeight < _levelHeight) {
    // System.out.println("Raising Elevator");
    // this._elevatorSubsytem.setSpeed(_upSpeed);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this._elevatorSubsytem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return _elevatorSubsytem.elevatorAtSetpoint();

    //return this._elevatorPIDController.atSetpoint();
    // return this._elevatorSubsytem.getAlgeaHeight() >= _levelHeight - 35
    // && this._elevatorSubsytem.getAlgeaHeight() <= _levelHeight + 35;
  }
}
