package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

public class Pneumatics extends SubsystemBase {
  private static Pneumatics instance;

   Compressor pcmCompressor = new Compressor(PneumaticConstants.compressorInput, PneumaticsModuleType.REVPH);
   private Solenoid _algeaSolenoid = new Solenoid(PneumaticsModuleType.REVPH, PneumaticConstants.algeaSolenoid);
   //private Solenoid _climberSolenoid = new Solenoid(PneumaticsModuleType.REVPH, PneumaticConstants.climberSolenoid);
      
  private Pneumatics() {
    pcmCompressor.enableAnalog(100,120);
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

  // public void climberOut() {
  //   this._climberSolenoid.set(true);
  // }
  
  // public void climberIn() {
  //   this._climberSolenoid.set(false);
  // }
}