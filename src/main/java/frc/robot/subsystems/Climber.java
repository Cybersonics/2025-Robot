// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private static Climber _instance;

  private SparkFlex _climberMotor;
  private SparkFlexConfig _climberConfig;

  /** Creates a new Climber. */
  public Climber() {
    _climberMotor = new SparkFlex(ClimberConstants.climberCANId, MotorType.kBrushless);
    _climberConfig = new SparkFlexConfig();
    _climberConfig.idleMode(IdleMode.kBrake);

        this._climberMotor.configure(_climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public static Climber getInstance() {
    if(_instance == null) {
      _instance = new Climber();
    }
    return _instance;
  }

  public void setSpeed(double speed) {
    _climberMotor.set(speed);
  }

  public void stop() {
    setSpeed(0);
  }

  public void positionClimber(double expectedPosition, boolean shouldRetract) {
    this._climberMotor.getEncoder().setPosition(expectedPosition);
  }

  public double getEncoderPosition() {
    return this._climberMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Encoder", getEncoderPosition());
  }
}