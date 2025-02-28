package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralMechanism;
import frc.robot.subsystems.Elevator;

public class IntakeCoralCommand extends Command {

    private CoralMechanism _coralSubsystems;
    private Elevator _elevator;
    private boolean _isFinished;

    public IntakeCoralCommand(CoralMechanism coral, Elevator elevator) {
        this._coralSubsystems = coral;
        this._elevator = elevator;

        addRequirements(_coralSubsystems, _elevator);
    }

    @Override
    public void initialize() {
        _isFinished = false;

    }

    @Override
    public void execute() {
        if (this._coralSubsystems.feedCoral()){         
            // if(this._elevator.getAlgeaHeight() <= 78) {
            //     this._elevator.setSpeed(.1);
            // } else {
            //     this._elevator.stop();
            // }           
            System.out.println("feeding Coral");
            this._coralSubsystems.intakeCoral();
        } else if (this._coralSubsystems.hasCoral()) {
            System.out.println("stopping Coral");
            this._coralSubsystems.stop(); 
            System.out.println("positioning Coral");
            this._coralSubsystems.setPosition(25);
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
