// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgeaConstants;

public class AlgeaMechanism extends SubsystemBase {

  private static AlgeaMechanism _instance;

  private SparkMax _algeaSparkMax;
  private SparkMaxConfig _algeaSparkConfig;
  
  /** Creates a new AlgeaMechanism. */
  private AlgeaMechanism() {
    this._algeaSparkMax = new SparkMax(AlgeaConstants.algeaMotorCANId, MotorType.kBrushless);

    this._algeaSparkConfig = new SparkMaxConfig();
    this._algeaSparkConfig.idleMode(IdleMode.kBrake);

    this._algeaSparkMax.configure(_algeaSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  /*
   * Setup Singleton for subsystem
   */
  public static AlgeaMechanism getInstance() {
    if(_instance == null) {
      _instance = new AlgeaMechanism();
    }
    return _instance;
  }

  public void setSpeed(double speed) {
    this._algeaSparkMax.set(speed);
  }

  public void intakeAlgea() {
    setSpeed(1);
  }

  public void ejectAlgea() {
    setSpeed(-1);
  }
  
  public void stop() {
    setSpeed(0);
  }

  private int spikeCount = 0;
  public boolean HasAmpCurrentSpiked() {
    if(_algeaSparkMax.getOutputCurrent() > 19) {
      if(spikeCount > 7) {  // Starting the intake spikes to 33amps look for x+ spikes before returning true
        spikeCount = 0;
        return true;
      } else {
        spikeCount += 1;
      }
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("Algea Amp Spike: "+_algeaSparkMax.getOutputCurrent());
    SmartDashboard.putNumber("Algea Amp Spike", _algeaSparkMax.getOutputCurrent());
  }
}
