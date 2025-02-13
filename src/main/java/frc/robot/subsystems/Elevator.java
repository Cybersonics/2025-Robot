// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private static Elevator _instance;

  private SparkFlex _leftMotor;
  private SparkFlexConfig _leftMotorConfig;
  private SparkFlex _rightMotor;
  private SparkFlexConfig _rightMotorConfig;

  private AnalogInput extPot;

  /** Creates a new Elevator. */
  private Elevator() {
    // Setup Motors
    _leftMotor = new SparkFlex(ElevatorConstants.leftMotorCANId, MotorType.kBrushless);
    _rightMotor = new SparkFlex(ElevatorConstants.rightMotorCANId, MotorType.kBrushless);

    // Setup String Pot    
    extPot = new AnalogInput(ElevatorConstants.stringPotChannel);

    //Configure Left motor
    _leftMotorConfig = new SparkFlexConfig();
    _leftMotorConfig.follow(_rightMotor);
    _leftMotorConfig.inverted(true);

    //Configure Right Motor
    _rightMotorConfig = new SparkFlexConfig();

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
