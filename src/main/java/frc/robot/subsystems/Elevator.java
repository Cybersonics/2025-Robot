// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class Elevator extends SubsystemBase {

  private static Elevator _instance;

  private SparkFlex _leftMotor;
  private SparkFlexConfig _leftMotorConfig;
  private SparkFlex _rightMotor;
  private SparkFlexConfig _rightMotorConfig;

  private AnalogInput _potInput;
  private AnalogPotentiometer _elevatorPosition;
  private double _levelHeight;
  private double maxSpeedUp = .7;
  private double slowedSpeed = .3;
  private double maxSpeedDown = .5;
  private PIDController _elevatorPIDController;
  private ElevatorFeedforward _feedforward;
  private double defaultPose; // = 0;


  private double outputVoltage;
	private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
			Constants.ElevatorConstants.ELEVATOR_MAX_VELO,
			Constants.ElevatorConstants.ELEVATOR_MAX_ACCELLERATION);

	private final ProfiledPIDController pidController = new ProfiledPIDController(Constants.ElevatorConstants.ELEVATOR_P,
			Constants.ElevatorConstants.ELEVATOR_I, Constants.ElevatorConstants.ELEVATOR_D, m_constraints);

	ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0086531, 0.15, 0.000215);//ks=0.0086531, kg=0.029608, kv=0.000215


  /** Creates a new Elevator. */
  private Elevator() {
    
    // Setup String Pot    
    _potInput = new AnalogInput(ElevatorConstants.stringPotChannel);

    _elevatorPosition = new AnalogPotentiometer(_potInput,95,0);

    // Setup Motors
    _leftMotor = new SparkFlex(ElevatorConstants.leftMotorCANId, MotorType.kBrushless);
    _rightMotor = new SparkFlex(ElevatorConstants.rightMotorCANId, MotorType.kBrushless);
    
    //Configure Left motor
    _leftMotorConfig = new SparkFlexConfig();
    _leftMotorConfig.follow(_rightMotor);
    _leftMotorConfig.idleMode(IdleMode.kBrake);
    _leftMotorConfig.openLoopRampRate(.5);

    //Configure Right Motor
    _rightMotorConfig = new SparkFlexConfig();
    _rightMotorConfig.idleMode(IdleMode.kBrake);
    _rightMotorConfig.openLoopRampRate(.5);

    //Flash Flex Controllers
    _leftMotor.configure(_leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    _rightMotor.configure(_rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Setup Elevator PIDs
    double p = 0.012;//  0.0093
    double i = 0;
    double d = 0.0005;//0.0025;//0.0005  0.0002

    double kS = 0.001;//0.01
    double kG = 0.01;//0.41
    double kV = 0.0002;//0.002
    _feedforward = new ElevatorFeedforward(kS, kG, kV);

    this._elevatorPIDController = new PIDController(p, i, d);
    this._elevatorPIDController.setTolerance(25);
    this.defaultPose=1000;


    pidController.setTolerance(Constants.ElevatorConstants.ELEVATOR_POSITION_TOLERANCE);
		pidController.reset(getPose());
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

  public void setSpeed(double speed) {
    SmartDashboard.putNumber("Current Elevator Position", getAlgeaHeight());
    SmartDashboard.putNumber("Current Elevator Speed", speed);
    this._leftMotor.set(speed);
    this._rightMotor.set(speed);
  }

  public void setVoltage(double volt) {
    this._leftMotor.setVoltage(volt);
    this._rightMotor.setVoltage(volt);
  }

  public double getEncoderVelocity() {
    return _leftMotor.getEncoder().getVelocity();
  }
  
  public void stop() {
    setSpeed(0);
  }

  public double getPose(){
    return this._potInput.getValue();
  }

	public void setDefaultPose(double pose){
		defaultPose = pose;
	}
	
  public void setPower(double power) {
		setVoltage(power * 12);
	}

  public double getDefaultPose(){
		return defaultPose;
	}

  public double getAlgeaHeight() {
    return this._potInput.getValue();
  }

  public void setElevatorPosition(double levelHeight){

    this._levelHeight = levelHeight;
    double currentHeight = getPose();
    //System.out.println("Moving from " + currentHeight + " to " + _levelHeight);

    double calcVoltage = this._elevatorPIDController.calculate(currentHeight, _levelHeight);
    SmartDashboard.putNumber("Current Elevator Voltage", calcVoltage);

    double feedForward = _feedforward.calculate(getEncoderVelocity());
    SmartDashboard.putNumber("Feed Forward Voltage", feedForward);

   setVoltage(calcVoltage + feedForward);

  }

  public void setElevatorHeight(double levelHeight){

   // System.out.println(pidController.getGoal().position+" , "+pidController.getSetpoint().position);

    setDefaultPose(levelHeight);
    pidController.setGoal(levelHeight);
      //this.outputVoltage = pidController.calculate(elevator.getPose(), elevator.getDefaultPose())//;
    this.outputVoltage = pidController.calculate(getPose())
    + elevatorFeedforward.calculate(pidController.getSetpoint().velocity);

    // SmartDashboard.putNumber("Current Elevator Pose", getPose());
    // SmartDashboard.putNumber("Default Elevator Pose", getDefaultPose());
    // SmartDashboard.putNumber("Elevator Power", this.outputVoltage);

    setVoltage(this.outputVoltage);
  }

  public void resetPosition(double position){
		pidController.reset(position);
	}

  public boolean elevatorAtSetpoint(){
    //return _elevatorPIDController.atSetpoint();
    return pidController.atSetpoint();
  }

  public boolean elevatorAtGoal(){
    return pidController.atGoal();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current Elevator Position", getAlgeaHeight());
    }
}
