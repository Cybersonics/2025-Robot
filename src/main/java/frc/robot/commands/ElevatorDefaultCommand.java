// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorDefaultCommand extends Command {
	private Elevator elevator;
  private CommandXboxController _opController;
	private double power;

  private double maxSpeedUp = .7;
  private double slowedSpeed = .3;
  private double maxSpeedDown = .5;

	private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
			Constants.ElevatorConstants.ELEVATOR_MAX_VELO,
			Constants.ElevatorConstants.ELEVATOR_MAX_ACCELLERATION);

	private final ProfiledPIDController pidController = new ProfiledPIDController(Constants.ElevatorConstants.ELEVATOR_P,
			Constants.ElevatorConstants.ELEVATOR_I, Constants.ElevatorConstants.ELEVATOR_D, m_constraints);

	ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0086531, 0.029608, 0.000215);


	public ElevatorDefaultCommand(CommandXboxController opController) {
		elevator = Elevator.getInstance();
    	this._opController = opController;
		this.power = 0;
		addRequirements(elevator);

		pidController.setTolerance(Constants.ElevatorConstants.ELEVATOR_POSITION_TOLERANCE);
	}

	@Override
	public void initialize() {
		elevator.setDefaultPose(elevator.getPose());
	}

	@Override
	public void execute() {

    double stickSpeed = -this._opController.getLeftY();

    if (Math.abs(stickSpeed)>0.01){
      if(this.elevator.getAlgeaHeight() < 3700 && stickSpeed > 0) {
        if(this.elevator.getAlgeaHeight() > 3000) {
            // Go slow close to top out
            this.elevator.setSpeed(stickSpeed * slowedSpeed);
        } else {
            // "full" speed when moving form bottom
            this.elevator.setSpeed(stickSpeed * maxSpeedUp);
        }
      } else if (this.elevator.getAlgeaHeight() > 95 && stickSpeed < 0) {
        // 70 is bottom stop before then let it settle on own with gravity.
        // Don't jam into bottom slow
        if (this.elevator.getAlgeaHeight() < 1200) {
            // Once below L2 slow down to bottom
            this.elevator.setSpeed(stickSpeed * slowedSpeed);
        } else {
            // "full" speed down from higher up
            this.elevator.setSpeed(stickSpeed * maxSpeedDown);
        }
      } else {
        // When above Soft Max or below Soft Low don't allow movement in that direction
        this.elevator.stop();
        //elevator.setDefaultPose(elevator.getPose());
      } 
      elevator.setDefaultPose(elevator.getPose());
    } else {
		  power = pidController.calculate(elevator.getPose(), elevator.getDefaultPose());
			  //+ elevatorFeedforward.calculate(pidController.getSetpoint().velocity);
		
			    SmartDashboard.putNumber("Current Elevator Pose", elevator.getPose());
				SmartDashboard.putNumber("Default Elevator Pose", elevator.getDefaultPose());
				SmartDashboard.putNumber("Elevator Power", power);


		  //if(pidController.getPositionError() < 0 && power < -0.4){
		  // 	power = -0.4;
		  // }

		  elevator.setPower(power);
    }
    
	}

	@Override
	public void end(boolean interrupted) {
		elevator.setPower(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
