package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command {

    private Elevator _elevatorSubsystems;
    private CommandXboxController _opController;

    public ElevatorCommand(Elevator elevator, CommandXboxController opController) {
        this._elevatorSubsystems = elevator;
        this._opController = opController;

        addRequirements(_elevatorSubsystems);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(this._opController.pov(0).getAsBoolean()) {
            this._elevatorSubsystems.setSpeed(.1);
        } else if (this._opController.pov(180).getAsBoolean()) {
            this._elevatorSubsystems.setSpeed(-.1);
        } else {
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
