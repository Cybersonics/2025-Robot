// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
public class NavXGyro extends SubsystemBase {

  private static NavXGyro instance;
  public static AHRS navX;
  public static double zeroHeading;
  public static double zeroAngle;

  /** Creates a new NavXGyro. */
  private NavXGyro() {
    navX = new AHRS(NavXComType.kMXP_SPI);

    zeroHeading = getGyroHeading();
    zeroAngle = getGyroAngle();
    //System.out.println("Setup ZeroAngle " + zeroAngle);
  }

  // Public Methods
  public static NavXGyro getInstance() {
    if (instance == null) {
      instance = new NavXGyro();
    }
    return instance;
  }

  public double getGyroHeading() {
    double heading = navX.getFusedHeading();
    return heading;
  }

  public double getGyroAngle() {
    double angle = navX.getAngle();
    return angle;
  }

  public void setGyroAngleOffset(double offset) {
    navX.setAngleAdjustment(offset);
  }

  public void zeroGyroHeading() {
    navX.reset();
    zeroHeading = getGyroHeading();
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
    //return Rotation2d.fromDegrees(navX.getAngle());
    return Rotation2d.fromDegrees(-navX.getAngle());
  }

  /*
   * Note that the math in the getHeading method is used to invert the direction
   * of
   * the gyro for use by wpilib which treats gyros backwards.
   * Gyros are normally clockwise positive. Wpilib wants
   * counter-clockwise positive.
   */
  public double getHeading() {
    return Math.IEEEremainder(-getGyroAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getPitchAngle() {
    return navX.getPitch();
  }
}