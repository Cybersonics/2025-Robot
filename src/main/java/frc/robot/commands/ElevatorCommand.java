package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command {

    private Elevator _elevatorSubsystem;
    private CommandXboxController _opController;

    private double maxSpeedUp = .7;
    private double slowedSpeed = .3;
    private double maxSpeedDown = .5;

    double kS = 0.005;//0.01
    double kG = 0.03;//0.41
    double kV = 0.0002;//0.002
    private ElevatorFeedforward _feedforward;

    public ElevatorCommand(Elevator elevator, CommandXboxController opController) {
        this._elevatorSubsystem = elevator;
        this._opController = opController;
        
        _feedforward = new ElevatorFeedforward(kS, kG, kV);

        addRequirements(_elevatorSubsystem);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        double stickSpeed = -this._opController.getLeftY();
        if(this._elevatorSubsystem.getAlgeaHeight() < 3700 && stickSpeed > 0) {
            if(this._elevatorSubsystem.getAlgeaHeight() > 3000) {
                // Go slow close to top out
                this._elevatorSubsystem.setSpeed(stickSpeed * slowedSpeed);
            } else {
                // "full" speed when moving form bottom
                this._elevatorSubsystem.setSpeed(stickSpeed * maxSpeedUp);
            }
        } else if (this._elevatorSubsystem.getAlgeaHeight() > 95 && stickSpeed < 0) {
            // 70 is bottom stop before then let it settle on own with gravity.
            // Don't jam into bottom slow
            if (this._elevatorSubsystem.getAlgeaHeight() < 1200) {
                // Once below L2 slow down to bottom
                this._elevatorSubsystem.setSpeed(stickSpeed * slowedSpeed);
            } else {
                // "full" speed down from higher up
                this._elevatorSubsystem.setSpeed(stickSpeed * maxSpeedDown);
            }
        } else {
            this._elevatorSubsystem.stop();
            // double feedForward = _feedforward.calculate(this._elevatorSubsystem.getEncoderVelocity());
            // SmartDashboard.putNumber("Feed Forward Voltage", feedForward);

            // this._elevatorSubsystem.setVoltage(feedForward);
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
