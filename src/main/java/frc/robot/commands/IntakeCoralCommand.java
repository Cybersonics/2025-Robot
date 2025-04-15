package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralMechanism;

public class IntakeCoralCommand extends Command {

    private CoralMechanism _coralSubsystems;
    private boolean _isFinished;

    public IntakeCoralCommand(CoralMechanism coral) {
        this._coralSubsystems = coral;

        addRequirements(_coralSubsystems);
    }

    @Override
    public void initialize() {
        _isFinished = false;

    }

    @Override
    public void execute() {
        if (this._coralSubsystems.feedCoral()){    
            System.out.println("feeding Coral");
            this._coralSubsystems.intakeCoral();
        } else if (this._coralSubsystems.hasCoral()) {
            System.out.println("stopping Coral");
            this._coralSubsystems.stop(); 
            System.out.println("positioning Coral");
            this._coralSubsystems.setPosition(15);
            this._isFinished = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this._coralSubsystems.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this._isFinished;
    }
}
