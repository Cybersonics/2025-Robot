package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralMechanism;

public class IntakeCoralCommand extends Command {

    private CoralMechanism _coralSubsystems;
    private Timer _feedTimer;
    private Timer _intakeTimer;
    private boolean _isFinished;

    public IntakeCoralCommand(CoralMechanism coral) {
        this._coralSubsystems = coral;

        addRequirements(_coralSubsystems);
    }

    @Override
    public void initialize() {
        _feedTimer = new Timer();
        _intakeTimer = new Timer();
        _isFinished = false;

    }

    @Override
    public void execute() {
        _feedTimer.start();
        if (this._coralSubsystems.hasCoral() && !this._coralSubsystems.feedCoral()) {
            System.out.println("stopping Coral");
            this._coralSubsystems.stop(); 
            System.out.println("positioning Coral");
            this._coralSubsystems.setPosition(10);
            this._isFinished = true;
        } else if (this._coralSubsystems.feedCoral() && _feedTimer.hasElapsed(.25)){                
            System.out.println("feeding Coral");
            this._coralSubsystems.intakeCoral();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this._feedTimer.stop();
        this._intakeTimer.stop();
        this._coralSubsystems.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this._isFinished;
    }
}
