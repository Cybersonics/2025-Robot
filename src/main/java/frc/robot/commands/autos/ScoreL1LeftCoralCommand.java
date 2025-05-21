package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralMechanism;

public class ScoreL1LeftCoralCommand extends Command {

    private CoralMechanism _coralSubsystems;

    public ScoreL1LeftCoralCommand(CoralMechanism coral) {
        this._coralSubsystems = coral;

        addRequirements(_coralSubsystems);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        this._coralSubsystems.levelOneEjectCoral(.45, .85);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this._coralSubsystems.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !this._coralSubsystems.hasCoral();
    }
}
