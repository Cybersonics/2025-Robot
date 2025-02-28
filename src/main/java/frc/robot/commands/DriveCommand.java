// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants.ModuleConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.PigeonGyro;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.AprilTag;
import frc.robot.utility.LimelightHelpers;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriveCommand extends Command {

  private Drive _drive;
  private CommandJoystick leftStick;
  private CommandJoystick rightStick;
  private CommandXboxController _driveController;
  //private XboxController _driveController_HID = _driveController.getHID();
  private JoystickButton _leftBumper;
  private JoystickButton _rightBumper;
    

  private NavXGyro _navXGyro;
  private PigeonGyro _pigeonGyro;
  private Camera _camera;
  private PhotonCamera _photonCamera;

  public static final double OMEGA_SCALE = 1.0 / 20.0;//30.0;// 45
  public static final double DEADZONE_LSTICK = 0.1;
  private static final double DEADZONE_RSTICK = 0.1;
  private double originHeading = 0.0;
  private double leftPow = 1;
  private double rightPow = 1;

  private PIDController _driveRotationPID, _driveDistancePID, _driveStrafePID;
  private double _driveRotationP = 0.0004, _driveRotationD = 0.00, _driveRotationI = 0.00;//p=0.0002
  private double _driveDistanceP = 0.015, _driveDistanceD = 0.008, _driveDistanceI = 0.00;//p=0.002
  private double _driveStrafeP = 0.015, _driveStrafeD = 0.00, _driveStrafeI = 0.00;//p=0.015 d=0.008
  private AprilTag _target;
  private double _aprilTagID;
  private PhotonTrackedTarget _bestTarget;

  private Trigger _rightJoystickButtonThree;

  /**
   * Creates a new DriveCommand using a standard set of joysticks as the driver joysticks.
   */
  public DriveCommand(Drive drive, CommandJoystick leftStick, CommandJoystick rightStick, NavXGyro gyro, Camera camera) {
    this._drive = drive;
    this.leftStick = leftStick;
    this.rightStick = rightStick;
    this._navXGyro = gyro;

    this._camera = camera;
    this._photonCamera = _camera.getPhotonCamera();
    
    _rightJoystickButtonThree = rightStick.button(3);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }  
  
  /**
  * Creates a new DriveCommand using a standard set of joysticks as the driver joysticks and Pigeon Gyro.
  */
 public DriveCommand(Drive drive, CommandJoystick leftStick, CommandJoystick rightStick, PigeonGyro gyro, Camera camera) {
   this._drive = drive;
   this.leftStick = leftStick;
   this.rightStick = rightStick;
   this._pigeonGyro = gyro;

   this._camera = camera;
   this._photonCamera = _camera.getPhotonCamera();
   
   _rightJoystickButtonThree = rightStick.button(3);

   // Use addRequirements() here to declare subsystem dependencies.
   addRequirements(drive);
 }

  /*
   * Creates a new DriveCommand using a CommandXboxController for driving
   */
  public DriveCommand(Drive drive, CommandXboxController driveController, NavXGyro gyro, Camera camera) {
    this._drive = drive;
    this._driveController = driveController;
    this._leftBumper = new JoystickButton(this._driveController.getHID(), XboxController.Button.kLeftBumper.value);
    this._rightBumper = new JoystickButton(this._driveController.getHID(), XboxController.Button.kRightBumper.value);
    this._navXGyro = gyro;
    
    this._camera = camera;
    this._photonCamera = _camera.getPhotonCamera();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }
  
  /*
  * Creates a new DriveCommand using a CommandXboxController and Pigeon Gyro for driving
  */
 public DriveCommand(Drive drive, CommandXboxController driveController, PigeonGyro gyro, Camera camera) {
   this._drive = drive;
   this._driveController = driveController;
   this._leftBumper = new JoystickButton(this._driveController.getHID(), XboxController.Button.kLeftBumper.value);
   this._rightBumper = new JoystickButton(this._driveController.getHID(), XboxController.Button.kRightBumper.value);
   this._pigeonGyro = gyro;
   
   this._camera = camera;
   this._photonCamera = _camera.getPhotonCamera();

   // Use addRequirements() here to declare subsystem dependencies.
   addRequirements(drive);
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*
     * Get the starting position of the gyro.
     * This will be used as the initial angle of the robot for field centric
     * control.
     */
    if(this._navXGyro != null) {
      originHeading = _navXGyro.getZeroAngle();
    } else if (this._pigeonGyro != null) {
      originHeading = _pigeonGyro.getZeroAngle();
    }
    // _drive.setDrivesMode(IdleMode.kCoast);
    _driveRotationPID = new PIDController(_driveRotationP, _driveRotationI, _driveRotationD);
    _driveRotationPID.setTolerance(0.8);

    _driveRotationPID.enableContinuousInput(-180,180);

    
    _driveDistancePID = new PIDController(_driveDistanceP, _driveDistanceI, _driveDistanceD);
    _driveDistancePID.setTolerance(2);

    _driveStrafePID = new PIDController(_driveStrafeP, _driveStrafeI, _driveStrafeD);
    _driveStrafePID.setTolerance(2);
    
    // _aprilTagID = LimelightHelpers.getFiducialID("");
    //_target = Constants.AprilTags.AprilTags.get(((int)LimelightHelpers.getFiducialID("")-1)); // indexed list is 0-15 not 1-16

    SmartDashboard.putNumber("Target Distance", 0);
    SmartDashboard.putNumber("Target Height", 0);
    SmartDashboard.putNumber("Target Heading", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // final double originOffset = 360 - originHeading;
    // originCorr = _navXGyro.getNavAngle() + originOffset;

    /*
     * The sticks are being inverted in the following lines to work with the
     * revised drive code. The revised drive code sets aligns the drive axis
     * and drive directions to match the odd directions of wpilib. This is done
     * to allow the use of path planning software in autonoumous mode.
     */

    double stickForward;
    double stickStrafe;
    double stickOmega;
    boolean deadStick = false;
    double speedMultiplier;
    if (_leftBumper.getAsBoolean()) {
      speedMultiplier = 1;
    }
    else if (_rightBumper.getAsBoolean()) {
      speedMultiplier = 0.25;
    }
    else {
      speedMultiplier = 0.50;
    }

    //stickForward = -this.leftStick.getY()*.95;
    //stickStrafe = -this.leftStick.getX()*.95;
    //stickOmega = -this.rightStick.getX()*.95;

    stickForward = -this._driveController.getLeftY()*speedMultiplier;
    stickStrafe = -this._driveController.getLeftX()*speedMultiplier;
    stickOmega = -this._driveController.getRightX()*.50;

    // SmartDashboard.putNumber("Controller Forward", stickForward);
    // SmartDashboard.putNumber("Controller Strafe", stickStrafe);
    // SmartDashboard.putNumber("Controller Omega", stickOmega);

    /*
     * The following lines allow the programmer to increase or decrease the initial
     * joystick action. The input of the joystick is taken to a power. If the
     * exponent is
     * one then the joystick action is linear. If the power is two then the initial
     * action will be a "soft" ramp, while the ending action will sharply increase.
     */

    double strafe = Math.pow(Math.abs(stickStrafe), leftPow) * Math.signum(stickStrafe);
    double forward = Math.pow(Math.abs(stickForward), leftPow) * Math.signum(stickForward);
    double omega = Math.pow(Math.abs(stickOmega), rightPow) * Math.signum(stickOmega) * OMEGA_SCALE;

    /*
     * If the input from the joystick is less than a dead zone value then set the
     * joystick output to zero. This prevents the robot from drifting due to the
     * joysticks
     * not fully returning to the zero position.
     * Note: take care when setting the deadzone value. If the value is set to a
     * high value,
     * the robot will move aggresively when the stick goes past the deadzone value.
     */
    if (Math.abs(strafe) < DEADZONE_LSTICK)
      strafe = 0.0;
    if (Math.abs(forward) < DEADZONE_LSTICK)
      forward = 0.0;
    if (Math.abs(omega) < DEADZONE_RSTICK * OMEGA_SCALE)
      omega = 0.0;
    boolean stickFieldCentric;
 
    stickFieldCentric = _driveController.leftTrigger().getAsBoolean();

     if (!stickFieldCentric) {

      /*
       * When the Left Joystick trigger is not pressed, The robot is in Field Centric
       * Mode.
       * The calculations correct the forward and strafe values for field centric
       * attitude.
       * Rotate the velocity vector from the joystick by the difference between our
       * current orientation and the current origin heading.
       */

      /*
       * Get the current angle of the robot and subtract it from the original heading
       * to determine
       * the angle of the robot to the field.
       */
      double originCorrection = 0;
      if(this._navXGyro != null){
        originCorrection = Math.toRadians(originHeading - this._navXGyro.getNavAngle());
      } else if (this._pigeonGyro != null) {
        originCorrection = Math.toRadians(originHeading - this._pigeonGyro.getHeading());
      }
  
      /*
       * Field centric code only affects the forward and strafe action, not rotation.
       * To perform field
       * centric movements we need to correct the forward and strafe action from the
       * joysticks by using
       * trig to determine how much each joystick movement contributes to moving in
       * the x and y axis of
       * the robot.
       */

      final double temp = forward * Math.cos(originCorrection) + strafe * Math.sin(originCorrection);
      strafe = strafe * Math.cos(originCorrection) - forward * Math.sin(originCorrection);
      forward = temp;
    }

   
    //var latestResult = _photonCamera.getLatestResult();
    
    /*if(_rightJoystickButtonThree.getAsBoolean()) {
      if(latestResult.hasTargets()) {
        _bestTarget = latestResult.getBestTarget();
        _aprilTagID = _bestTarget.getFiducialId();
         _aprilTagID = LimelightHelpers.getFiducialID("");
      }
      if (_aprilTagID>-1){
        AprilTag target = Constants.AprilTags.AprilTags.get(((int)_aprilTagID-1)); // indexed list is 0-15 not 1-16
        double targetDistance = target.getDistance();
        double targetHeight = target.getHeight();
        double targetHeading = target.getExpectedHeading();

        SmartDashboard.putNumber("Target Distance", target.getDistance());
        SmartDashboard.putNumber("Target Height", target.getHeight());
        SmartDashboard.putNumber("Target Heading", target.getExpectedHeading());
    
    
        //double rotationEstimate = LimelightHelpers.getTY("");// + Constants.TrapConstants.AngleOffset;
        //double rotationEstimate = LimelightHelpers.getTX("");// + Constants.TrapConstants.AngleOffset;
        //double rotationValue = _driveRotationPID.calculate(rotationEstimate, 0);
        double rotationValue = _driveRotationPID.calculate(-_navXGyro.getNavAngle(), targetHeading);
        
        //SmartDashboard.putNumber("Rotation Estimate", rotationEstimate);
        SmartDashboard.putNumber("Rotation Value", rotationValue);


        //How many degrees back is limelight rotated from vertical
        /*Vertical angle calculated by setting bot a fixed distance back from target (measureDistanceToTarget) with 
        height of target and height of camera lens measured in inches.
        Use a calculator to get the Total Angle = arcTan(targetHeight-cameraHeight)/measuredDistanceToTaget
        Using the Limelight webviewer get the ty value. Take the total angle calculated above and subtract the 
        ty value from the Limelight webvier to get the limelightMountAngleDegrees.
        */

        /*double tx = _bestTarget.getYaw();
        // double tx = LimelightHelpers.getTX("");

        //Vertical angle of target in view in degrees
        double ty = _bestTarget.getPitch();
        // double ty = LimelightHelpers.getTY(""); 

        double limelightMountAngleDegrees = 30.6; //29.085;//32; 

        //Distance from center of limelight lens to floor
        double limelightLensHeightInches = 14.75; //14.5;

        //Distance fron target to floor
        //double goalHeightInches = 52.0;//50.5;

        double angleToGoalDegrees = limelightMountAngleDegrees + ty;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //Calculate distance
        double distanceFromLimelight = (targetHeight - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        SmartDashboard.putNumber("Distance Value", distanceFromLimelight);

        double distanceValue = _driveDistancePID.calculate(distanceFromLimelight, targetDistance);

        double strafeValue = _driveStrafePID.calculate(tx, 0);

        // This isn't correct either since it doesn't account for the vector movement its only forward/reverse 
        // and its at the same time as rotaiton maybe we should separate it?     
        // _drive.processInput(distanceValue, 0.0, -rotationValue, false);

        //_drive.processInput(distanceValue, -strafeValue, rotationValue, false); //-rotationValue
        forward = distanceValue;
        strafe = -strafeValue;
        omega = rotationValue;
        deadStick = false;
      } else {
        // When losing april tag kill all movement.
        forward = 0;
        strafe = 0;
        omega = 0;
        deadStick = true;
      }
    }/* */

    /*
      * If all of the joysticks are in the deadzone, don't update the motors
    */


    if (strafe == 0.0 && forward == 0.0 && omega == 0.0) {
      deadStick = true;
    }

    // SmartDashboard.putNumber("Forward Done", forward);
    // SmartDashboard.putNumber("Strafe Done", strafe);
    // SmartDashboard.putNumber("Rotation Done", omega);
    SmartDashboard.putBoolean("Dead Stick", deadStick);

    /*
     * Take the calculated values from the joysticks and use the values to operate
     * the drive system.
    */

    this._drive.processInput(forward, strafe, omega, deadStick);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}