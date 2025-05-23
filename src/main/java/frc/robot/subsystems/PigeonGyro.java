package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class PigeonGyro extends SubsystemBase {

    private static PigeonGyro instance;
    
    public static Pigeon2 pigeon;
    public static double zeroHeading;
    public static double zeroAngle;
  
    /** Creates a new PigeonGyro. */
    private PigeonGyro() {
      pigeon = new Pigeon2(DriveConstants.PigeonCANId);
  
      zeroHeading = getGyroHeading();
      zeroAngle = getGyroAngle();
      //System.out.println("Setup ZeroAngle " + zeroAngle);
    }
  
    // Public Methods
    public static PigeonGyro getInstance() {
      if (instance == null) {
        instance = new PigeonGyro();
      }
      return instance;
    }
  
    public double getGyroYawValue() {
      double heading = pigeon.getYaw().getValueAsDouble();
      return heading;
    }

    public double getGyroHeading() {
      double heading = pigeon.getYaw().getValueAsDouble();
      return heading;
    }
  
    public double getGyroAngle() {
      double angle = pigeon.getYaw().getValueAsDouble();
      return angle;
    }
  
    public void setGyroAngleOffset(double offset) {
      pigeon.setYaw(offset);
    }
  
    public void zeroGyroHeading() {
      pigeon.reset();
      //zeroHeading = getGyroHeading();
      zeroAngle = getGyroAngle();
      System.out.println("ZeroHeading: " + zeroHeading);
      System.out.println("ZeroAngle: " + zeroAngle);
    }
  
    public double getGyroZeroHeading() {
      return zeroHeading;
    }
  
    public double getGyroZeroAngle() {
      return zeroAngle;
    }
  
    public Rotation2d getGyroRotation2D() {
      return pigeon.getRotation2d();
      //return Rotation2d.fromDegrees(getHeading());
    }
  
    /*
     * Note that the math in the getHeading method is used to invert the direction
     * of
     * the gyro for use by wpilib which treats gyros backwards.
     * Gyros are normally clockwise positive. Wpilib wants
     * counter-clockwise positive.
     */
    public double getHeading() {
      //return Math.IEEEremainder(-getNavAngle(), 360);
      return Math.IEEEremainder(getGyroAngle(), 360);
    }
  
    public Rotation2d getRotation2d() {
      return Rotation2d.fromDegrees(getHeading());
    }
  
    public double getPitchAngle() {
      return pigeon.getPitch().getValueAsDouble();
    }
  }