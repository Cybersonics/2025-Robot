// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PneumaticConstants;

public class Pneumatics extends SubsystemBase {
  private static Pneumatics instance;

   Compressor pcmCompressor = new Compressor(PneumaticConstants.compressorInput, PneumaticsModuleType.REVPH);
   private Solenoid _algeaSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 15);
      
  /** Creates a new Pneumatics. */
  private Pneumatics() {
    pcmCompressor.enableAnalog(80,100);
  }

  public static Pneumatics getInstance() {
    if(instance == null) {
      instance = new Pneumatics();
    }
    return instance;
  }

  public void algeaOut() {
    this._algeaSolenoid.set(true);
  }
  
  public void algeaIn() {
    this._algeaSolenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
