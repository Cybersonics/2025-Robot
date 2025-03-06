/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig.Type;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.MathUtil; // Use for RoboRio PID
import edu.wpi.first.math.controller.PIDController; //Use for Roborio PID
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.utility.SparkMaxUtil;
import frc.robot.Constants.shuffleBoardDrive;

public class SwerveModule extends SubsystemBase {

  /**
   * Creates a new swerveModule.
   */

  public double currentPosition;

  private SparkFlex driveMotor;
  private SparkFlexConfig driveConfig;

  private SparkFlexConfig steerConfig;
  private SparkFlex steerMotor;

  private RelativeEncoder driveMotorEncoder; // Set up integrated Drive motor encoder in Spark Max/Neo
  private AbsoluteEncoder steerMotorEncoder; // Set up integrated Steer motor encoder in Spark Max/550

  private final SparkClosedLoopController drivePIDController;
  private final SparkClosedLoopController steerPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private static final double RAMP_RATE = 0.5;
  // private static final double STEER_P = 3.0, STEER_I = 0.0, STEER_D = 0.1;

  public double encoderCountPerRotation = 1024;

  private boolean _driveCorrect;

  private shuffleBoardDrive driveData;
  private ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTab");
  private GenericEntry setAngleOffset;

  public SwerveModule(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer,
      shuffleBoardDrive driveData) {

    // Get Drive information from Constants and create a Drive Alignment Tuner on
    // ShuffleBoard
    this.driveData = driveData;

    // setAngleOffset = driveTab.addPersistent(this.driveData.drivePosition, 0)
    // .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180,
    // "max", 180, "center", 0))
    // .withPosition(this.driveData.colPos, this.driveData.rowPos).withSize(3,
    // 1).getEntry();

    // Create and configure a new Drive motor
    driveMotor = new SparkFlex(driveNum, MotorType.kBrushless);
    driveConfig = new SparkFlexConfig();
    driveConfig.openLoopRampRate(RAMP_RATE);// This provides a motor ramp up time to prevent brown outs.
    driveConfig.idleMode(IdleMode.kCoast);
    driveConfig.smartCurrentLimit(55);
    driveConfig.inverted(invertDrive);// setInverted reverses the both the motor and the encoder direction.

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    driveConfig.encoder.positionConversionFactor(DriveConstants.ModuleConstants.kDrivingEncoderPositionFactor);
    driveConfig.encoder.velocityConversionFactor(DriveConstants.ModuleConstants.kDrivingEncoderVelocityFactor);
    driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // Set the PID gains for the driving motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    driveConfig.closedLoop.pidf(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD,
        ModuleConstants.kDrivingFF);
    driveConfig.closedLoop.outputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Create and configure a new Steering motor
    steerMotor = new SparkFlex(steerNum, MotorType.kBrushless);
    steerConfig = new SparkFlexConfig();
    steerConfig.inverted(invertSteer);
    steerConfig.idleMode(IdleMode.kBrake);
    steerConfig.smartCurrentLimit(30);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    steerConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    steerConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    steerConfig.absoluteEncoder.inverted(ModuleConstants.kTurningEncoderInverted);

    steerConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    steerConfig.closedLoop.pidf(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD,
        ModuleConstants.kTurningFF);
    steerConfig.closedLoop.outputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);
    steerMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    steerConfig.closedLoop.positionWrappingEnabled(true);
    steerConfig.closedLoop.positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    steerConfig.closedLoop.positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create the built in motor encoders
    driveMotorEncoder = driveMotor.getEncoder();
    drivePIDController = driveMotor.getClosedLoopController();

    // Create the built in motor encoders
    steerMotorEncoder = steerMotor.getAbsoluteEncoder();
    steerPIDController = steerMotor.getClosedLoopController();
  }

  public void setSwerve(double angle, double speed, boolean driveCorrect) {

    this._driveCorrect = driveCorrect;
    double scaledPosition;
 
      /*
       * Get the current angle of the absolute encoder in raw encoder format and
       * then convert the raw angle into degrees. Use the Modulus % function
       * to find the current pivot position inside of the 0 to 360 degree range.
       * Get the offset angle from the dashboard to "trim" the position of the pivots.
       * 
       * The target angle is then used to calculate how far the pivot needs to turn
       * based on the difference of the target angle and the current angle.
       */

      double currentSteerPosition = rawEncoderPosition();
      double currentAngle = ((currentSteerPosition * 360) / (2 * Math.PI)) % 360.0;

      // double targetAngle = -angle + getAngleOffset(); // -angle;
      double targetAngle = angle; // -angle;
      double deltaDegrees = targetAngle - currentAngle;

      // SmartDashboard.putNumber(this.driveData.drivePosition + " Raw Angle",
      // currentAngle);
      // SmartDashboard.putNumber(this.driveData.drivePosition + " Offset Angle",
      // getAngleOffset());
      // SmartDashboard.putNumber(this.driveData.drivePosition + " cur Angle",
      // targetAngle);
      /*
       * The encoder reads in degrees from 0 to 360 where the 0/360 degree position is
       * straight ahead.
       * The swerve equations generate position angles from -180 to 180 degrees where
       * the
       * center point of zero degrees is straight ahead.
       * To relate the two coordinate systems we "shift" the encoder reading to make
       * it "read" -180 to 180
       */
      if (Math.abs(deltaDegrees) > 180.0) {
        deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
      }
    if (!this._driveCorrect) {
      /*
       * If we need to turn more than 90 degrees, we can reverse the wheel direction
       * instead and only rotate by the complement
       */
      // if (Math.abs(speed) <= MAX_SPEED){
      if (Math.abs(deltaDegrees) > 90.0) {
        deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
        speed = -speed;
      }
     }

      // Add change in position to current position
      double targetPosition = currentAngle + deltaDegrees;
      // Scale the new position to match the motor encoder
      scaledPosition = (targetPosition * (2 * Math.PI) / 360);
   // } else {
     // scaledPosition = (angle * (2 * Math.PI) / 360);
    

    steerPIDController.setReference(scaledPosition, SparkMax.ControlType.kPosition);

    // SmartDashboard.putNumber(this.driveData.drivePosition + " SSpeed",
    // scaledPosition);
    // SmartDashboard.putNumber(this.driveData.drivePosition + " ZeroOffset",
    // steerMotorEncoder.getZeroOffset());

    driveMotor.set(speed);

    // Use Dashboard items to help debug
    // SmartDashboard.putNumber(this.driveData.drivePosition+" Incoming Angle",
    // angle);
    // SmartDashboard.putNumber(this.driveData.drivePosition+" CurAngle",
    // currentAngle);
    // SmartDashboard.putNumber(this.driveData.drivePosition+" TargetAngle",
    // targetAngle);
    // SmartDashboard.putNumber(this.driveData.drivePosition+"
    // currentSteerPosition", currentSteerPosition);
    // SmartDashboard.putNumber("DeltaDegrees", deltaDegrees);
    // SmartDashboard.putNumber("TargetPosition", targetPosition);
    // SmartDashboard.putNumber("Steer Output", scaledPosition);
    // SmartDashboard.putNumber("currentPosition", currentAngle);
    // SmartDashboard.putNumber("Steer Output", steerOutput);
  }

  /*
   * Get the built in Spark/Neo Drive motor encoder position.
   */
  public double getDriveEncoder() {
    return driveMotorEncoder.getPosition();
  }

  /*
   * Set the position value of the Spark/Neo Drive motor encoder position.
   */
  public void setDriveEncoder(double position) {
    driveMotorEncoder.setPosition(position);
  }

  public double getDriveVelocity() {
    return driveMotorEncoder.getVelocity();
  }

  /*
   * Set the drive motor speed from -1 to 1
   */
  public void setDriveSpeed(double speed) {
    driveMotor.set(speed);
  }

  /*
   * Get the drive motor speed.
   */
  public double getDriveSpeed() {
    return driveMotor.get();
  }

  public void stopDriveMotor() {
    driveMotor.stopMotor();
  }

  /*
   * Set the steer motor speed from -1 to 1
   */
  public void setSteerSpeed(double speed) {
    steerMotor.set(speed);
  }

  /*
   * Get the "Trimming" offset for the pivot from the dashboard.
   * The trimming offset is used to fine tune the pivot angles
   * so they are all facing the same way.
   */
  public double getAngleOffset() {
    double angleOffset = setAngleOffset.getDouble(0.0);
    return angleOffset;
  }

  public double rawEncoderPosition() {
    return steerMotorEncoder.getPosition();
  }

  /*
   * Get the raw pivot angle and add the "trimming" offset from the
   * dashboard. Convert the pivot angle to radians to be used by the
   * autonomous routines. The getState and getPosition are used
   * by the SweveModuleState system employed by WPILib.
   */
  public double getTurningPosition() {
    double steerEncoderRaw = rawEncoderPosition();
    // double angleOffset = (getAngleOffset() / 360) *2 * Math.PI;
    // double turningEncoder = steerEncoderRaw + angleOffset;
    double turningEncoder = steerEncoderRaw;
    return turningEncoder; // -turningEncoder Invert Encoder for odometry as wpilib treats encoders
                           // backwards.
  }

  public void resetEncoders() {
    driveMotorEncoder.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveEncoder(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);
    double driveMotorSpeed = state.speedMetersPerSecond
        / DriveConstants.FrameConstants.kPhysicalMaxSpeedMetersPerSecond;
    double steerMotorAngle = MathUtil.inputModulus(state.angle.getDegrees(), -180, 180);
    setSwerve(steerMotorAngle, driveMotorSpeed, true);
  }

  public void stop() {
    driveMotor.set(0);
  }

  public void driveMotorRamp(boolean enableRamp) {
    if (enableRamp) {
      driveConfig.openLoopRampRate(RAMP_RATE);
      driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    } else {
      driveConfig.openLoopRampRate(0);
      driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  // Set Drive Mode
  public void setDriveMode(IdleMode idleMode) {
    driveConfig.idleMode(idleMode);
    driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // Get Drive Mode
  public IdleMode getDriveMode() {
    return driveMotor.configAccessor.getIdleMode();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
