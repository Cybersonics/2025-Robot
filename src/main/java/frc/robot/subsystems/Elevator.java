// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private static Elevator _instance;

  private SparkFlex _leftMotor;
  private SparkFlexConfig _leftMotorConfig;
  private SparkFlex _rightMotor;
  private SparkFlexConfig _rightMotorConfig;

  private AnalogInput _potInput;
 

  /** Creates a new Elevator. */
  private Elevator() {
    // Setup Motors
    _leftMotor = new SparkFlex(ElevatorConstants.leftMotorCANId, MotorType.kBrushless);
    _rightMotor = new SparkFlex(ElevatorConstants.rightMotorCANId, MotorType.kBrushless);

    // Setup String Pot    
    _potInput = new AnalogInput(ElevatorConstants.stringPotChannel);
    
    //Configure Left motor
    _leftMotorConfig = new SparkFlexConfig();
    _leftMotorConfig.follow(_rightMotor);
    _leftMotorConfig.idleMode(IdleMode.kBrake);
    _leftMotorConfig.openLoopRampRate(1);

    //Configure Right Motor
    _rightMotorConfig = new SparkFlexConfig();
    _rightMotorConfig.idleMode(IdleMode.kBrake);
    _rightMotorConfig.openLoopRampRate(1);

    //Flash Flex Controllers
     _leftMotor.configure(_leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     _rightMotor.configure(_rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /*
   * Setup Singleton for subsystem
   */
  public static Elevator getInstance() {
    if(_instance == null) {
      _instance = new Elevator();
    }
    return _instance;
  }

  public void setSpeed(double speed) {
    SmartDashboard.putNumber("Current Elevator Position", getAlgeaHeight());
    this._leftMotor.set(speed);
    this._rightMotor.set(speed);
  }

  public void stop() {
    setSpeed(0);
  }

  public double getAlgeaHeight() {
    return this._potInput.getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current Elevator Position", getAlgeaHeight());
    }
}
