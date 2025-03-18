package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private static Climber _instance;

  private SparkFlex _climberMotor;
  private SparkFlexConfig _climberConfig;

  private Climber() {
    this._climberMotor = new SparkFlex(ClimberConstants.climberCANId, MotorType.kBrushless);
    this._climberConfig = new SparkFlexConfig();
    this._climberConfig.idleMode(IdleMode.kBrake);
    
    this._climberMotor.configure(_climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public static Climber getInstance() {
    if(_instance == null) {
      _instance = new Climber();
    }
    return _instance;
  }

  public void setSpeed(double speed) {
    this._climberMotor.set(speed);
  }

  public void retractClimber() {
    setSpeed(.5);
  }

  public void extendClimber() {
    setSpeed(-.5);
  }
}
