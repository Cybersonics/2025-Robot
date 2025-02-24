// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralMechanism extends SubsystemBase {

  private static CoralMechanism _instance;

  private SparkMax _coralLeftSparkMax;
  private SparkMaxConfig _coralLeftSparkConfig;

  private SparkMax _coralRightSparkMax;
  private SparkMaxConfig _coralRightSparkConfig;
  
  private DigitalInput _coralTrip;  
  private DigitalInput _coralFeed;  

  /** Creates a new CoralMechanism. */
  private CoralMechanism() {
    this._coralTrip = new DigitalInput(CoralConstants.coralTripSensorId);
    this._coralFeed = new DigitalInput(CoralConstants.coralFeedSesorId);

    this._coralLeftSparkMax = new SparkMax(CoralConstants.leftCoralMotorCANId, MotorType.kBrushless);
    this._coralRightSparkMax = new SparkMax(CoralConstants.rightCoralMotorCANId, MotorType.kBrushless);

    this._coralLeftSparkConfig = new SparkMaxConfig();
    this._coralLeftSparkConfig.idleMode(IdleMode.kBrake);

    this._coralRightSparkConfig = new SparkMaxConfig();
    this._coralRightSparkConfig.idleMode(IdleMode.kBrake);
    this._coralRightSparkConfig.inverted(true);

    this._coralLeftSparkMax.configure(_coralLeftSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this._coralRightSparkMax.configure(_coralRightSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /*
   * Setup Singleton for subsystem
   */
  public static CoralMechanism getInstance() {
    if (_instance == null) {
      _instance = new CoralMechanism();
    }
    return _instance;
  }

  public void setPosition(double positionIncrease) {
    var leftIncrease = this._coralLeftSparkMax.getEncoder().getPosition() + positionIncrease;
    this._coralLeftSparkMax.getEncoder().setPosition(leftIncrease);
    
    var rightIncrease = this._coralRightSparkMax.getEncoder().getPosition() + positionIncrease;
    this._coralRightSparkMax.getEncoder().setPosition(rightIncrease);
  }

  public void setSpeed(double speed) {
    this._coralRightSparkMax.set(speed);
    this._coralLeftSparkMax.set(speed);
  }

  public void intakeCoral() {
    setSpeed(.5);
  }

  public void ejectCoral() {
    setSpeed(-.5);
  }

  public boolean hasCoral() {
    return !this._coralTrip.get();
  }
  
  public boolean feedCoral() {
    return !this._coralFeed.get();
  }

  public void stop() {
    setSpeed(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run    
    SmartDashboard.putBoolean("MechanismHasCoral", hasCoral());   
    SmartDashboard.putBoolean("FeedHasCoral", feedCoral());

    SmartDashboard.putNumber("LeftEncoder", this._coralLeftSparkMax.getEncoder().getPosition());
    SmartDashboard.putNumber("RightEncoder", this._coralRightSparkMax.getEncoder().getPosition());
  }
}
