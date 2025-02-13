// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralMechanism extends SubsystemBase {

  private static CoralMechanism _instance;

  /** Creates a new CoralMechanism. */
  private CoralMechanism() {}

  /*
   * Setup Singleton for subsystem
   */
  public static CoralMechanism getInstance() {
    if(_instance == null) {
      _instance = new CoralMechanism();
    }
    return _instance;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
