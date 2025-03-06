package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command {

    private Elevator _elevatorSubsystems;
    private CommandXboxController _opController;

    private double maxSpeedUp = .7;
    private double slowedSpeed = .3;
    private double maxSpeedDown = .5;

    public ElevatorCommand(Elevator elevator, CommandXboxController opController) {
        this._elevatorSubsystems = elevator;
        this._opController = opController;

        addRequirements(_elevatorSubsystems);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        double stickSpeed = -this._opController.getLeftY();
            if(this._elevatorSubsystems.getAlgeaHeight() < 3700 && stickSpeed > 0) {
                if(this._elevatorSubsystems.getAlgeaHeight() > 3000) {
                    // Go slow close to top out
                    this._elevatorSubsystems.setSpeed(stickSpeed * slowedSpeed);
                } else {
                    // "full" speed when moving form bottom
                    this._elevatorSubsystems.setSpeed(stickSpeed * maxSpeedUp);
                }
            } else if (this._elevatorSubsystems.getAlgeaHeight() > 95 && stickSpeed < 0) {
                // 70 is bottom stop before then let it settle on own with gravity.
                // Don't jam into bottom slow
                if (this._elevatorSubsystems.getAlgeaHeight() < 1200) {
                    // Once below L2 slow down to bottom
                    this._elevatorSubsystems.setSpeed(stickSpeed * slowedSpeed);
                } else {
                    // "full" speed down from higher up
                    this._elevatorSubsystems.setSpeed(stickSpeed * maxSpeedDown);
                }
            } else {
                // When above Soft Max or below Soft Low don't allow movement in that direction
                this._elevatorSubsystems.stop();
            }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
