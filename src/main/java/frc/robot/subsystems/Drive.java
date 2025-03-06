package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

public class Drive extends SubsystemBase {

	private static Drive instance;

	//public static final double kMaxSpeed = 3.0; // 3 meters per second
	//public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

	private static SwerveModule frontLeft;
	private static SwerveModule backLeft;
	private static SwerveModule frontRight;
	private static SwerveModule backRight;

	public double heading;
	public double angle;

	public static final double OMEGA_SCALE = 1.0 / 30.0;

	private final boolean invertDrive = false;//true;
	private final boolean invertSteer = true; //true;
	private NavXGyro _navXGyro;
	private PigeonGyro _pigeonGyro;
	private boolean _driveCorrect;

	private final SwerveDriveOdometry odometer;

	/*
	 * Set up the drive by passing in the gyro and then configuring the individual
	 * swerve modules.
	 * Note the order that the modules are in. Be consistant with the order in the
	 * odometry.
	 */
	private Drive(PigeonGyro gyro) {

		this._pigeonGyro = gyro;

		frontLeft = new SwerveModule(DriveConstants.FrontLeftSteer, DriveConstants.FrontLeftDrive, invertDrive,
				invertSteer);

		frontRight = new SwerveModule(DriveConstants.FrontRightSteer, DriveConstants.FrontRightDrive, invertDrive,
		 		invertSteer);

		backLeft = new SwerveModule(DriveConstants.BackLeftSteer, DriveConstants.BackLeftDrive, invertDrive,
		 		invertSteer);

		backRight = new SwerveModule(DriveConstants.BackRightSteer, DriveConstants.BackRightDrive, invertDrive,
		 		invertSteer);

		 odometer = new SwerveDriveOdometry(DriveConstants.FrameConstants.kDriveKinematics,
		 		this._pigeonGyro.getGyroRotation2D(), getPositions());

		RobotConfig robotConfig;
		try{
			robotConfig = RobotConfig.fromGUISettings();
			
			// Configure AutoBuilder
			AutoBuilder.configure(
					this::getPose,
					this::resetPose,
					this::getSpeeds,
					this::driveRobotRelative,
					DriveConstants.pathFollowerConfig,
					robotConfig,
					() -> {
						// Boolean supplier that controls when the path will be mirrored for the red alliance
						// This will flip the path being followed to the red side of the field.
						// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

						var alliance = DriverStation.getAlliance();
						if (alliance.isPresent()) {
							return alliance.get() == DriverStation.Alliance.Red;
						}
						return false;
					},
					this
				);
		} catch (Exception e) {
			// Handle exception as needed
			e.printStackTrace();
		}
	}

	/*
	 * Set up the drive by passing in the gyro and then configuring the individual
	 * swerve modules.
	 * Note the order that the modules are in. Be consistant with the order in the
	 * odometry.
	 */
	private Drive(NavXGyro gyro) {

		this._navXGyro = gyro;

		frontLeft = new SwerveModule(DriveConstants.FrontLeftSteer, DriveConstants.FrontLeftDrive, invertDrive,
				invertSteer);

		frontRight = new SwerveModule(DriveConstants.FrontRightSteer, DriveConstants.FrontRightDrive, invertDrive,
		 		invertSteer);

		backLeft = new SwerveModule(DriveConstants.BackLeftSteer, DriveConstants.BackLeftDrive, invertDrive,
		 		invertSteer);

		backRight = new SwerveModule(DriveConstants.BackRightSteer, DriveConstants.BackRightDrive, invertDrive,
		 		invertSteer);

		 odometer = new SwerveDriveOdometry(DriveConstants.FrameConstants.kDriveKinematics,
		 		this._navXGyro.getGyroRotation2D(), getPositions());

		RobotConfig robotConfig;
		try{
			robotConfig = RobotConfig.fromGUISettings();
			
			// Configure AutoBuilder
			AutoBuilder.configure(
					this::getPose,
					this::resetPose,
					this::getSpeeds,
					this::driveRobotRelative,
					DriveConstants.pathFollowerConfig,
					robotConfig,
					() -> {
						// Boolean supplier that controls when the path will be mirrored for the red alliance
						// This will flip the path being followed to the red side of the field.
						// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

						var alliance = DriverStation.getAlliance();
						if (alliance.isPresent()) {
							return alliance.get() == DriverStation.Alliance.Red;
						}
						return false;
					},
					this
				);
		} catch (Exception e) {
			// Handle exception as needed
			e.printStackTrace();
		}
	}

	// Public Methods

	public Pose2d getPose() {
		return odometer.getPoseMeters();
	}

	public void resetPose(Pose2d pose) {
		if(this._navXGyro != null) {
			odometer.resetPosition(this._navXGyro.getGyroRotation2D(), getPositions(), pose);
		} else if (this._pigeonGyro != null) {
			odometer.resetPosition(this._pigeonGyro.getGyroRotation2D(), getPositions(), pose);
		}
	}

	public ChassisSpeeds getSpeeds() {
		return DriveConstants.FrameConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
	}

	public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
		driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
	}

	public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {	
		ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

		SwerveModuleState[] targetStates = DriveConstants.FrameConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
		setModuleStates(targetStates);
	}

	public void resetOdometry(Pose2d pose) {
		if(this._navXGyro != null) {
			odometer.resetPosition(this._navXGyro.getRotation2d(), getPositions(), pose);
		} else if (this._pigeonGyro != null) {
			odometer.resetPosition(this._pigeonGyro.getRotation2d(), getPositions(), pose);
		}
	}

	// public void resetOdometryForState(PathPlannerState state) {
	// state = PathPlannerTrajectory.transformStateForAlliance(state, DriverStation.getAlliance());
	// Pose2d pose = new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
	// odometer.resetPosition(this._gyro.getRotation2d(), getPositions(), pose);
	// }

	public static Drive getInstance(NavXGyro gyro) {
		if (instance == null) {
			instance = new Drive(gyro);
		}
		return instance;
	}

	public static Drive getInstance(PigeonGyro gyro) {
		if (instance == null) {
			instance = new Drive(gyro);
		}
		return instance;
	}

	public void processInput(double forward, double strafe, double omega, boolean deadStick) {

		double omegaL2 = omega * (DriveConstants.FrameConstants.WHEEL_BASE_LENGTH / 2.0);
		double omegaW2 = omega * (DriveConstants.FrameConstants.WHEEL_BASE_WIDTH / 2.0);

		this._driveCorrect = false;
		
		SmartDashboard.putNumber("OmegaL2", omegaL2);
		SmartDashboard.putNumber("OmegaW2", omegaW2);
		SmartDashboard.putNumber("Forward", forward);
		SmartDashboard.putNumber("Strafe", strafe);
		// SmartDashboard.putNumber("NavX", _gyro.getNavAngle());
		// Compute the constants used later for calculating speeds and angles
		double A = strafe - omegaL2;
		double B = strafe + omegaL2;
		double C = forward - omegaW2;
		double D = forward + omegaW2;

		/*
		 * Compute the drive motor speeds
		 * Constant values re-arranged to invert direction of drive controls
		 * to work with inverted wpilib paths.
		 * Positive Y is now left. Positive X is forward. Positive rotation is
		 * counter-clockwise.
		 */
		double speedFL = speed(B, C);
		double speedBL = speed(A, C);
		double speedFR = speed(B, D);
		double speedBR = speed(A, D);

		/*
		 * Compute the steer motor positions
		 * Constant values re-arranged to invert direction of steer motor controls
		 * to work with inverted wpilib paths.
		 * Positive Y is now left. Positive X is forward. Positive rotation is
		 * counter-clockwise.
		 * NOTE: The letter sets in the speed and angle sections MUST MATCH
		 */
		double angleFL = angle(B, C);
		double angleBL = angle(A, C);
		double angleFR = angle(B, D);
		double angleBR = angle(A, D);

		/*
		 * Compute the maximum speed so that we can scale all the speeds to the range
		 * [0.0, 1.0]
		 */
		double maxSpeed = Collections.max(Arrays.asList(speedFL, speedBL, speedFR, speedBR, 1.0));

		
		if (deadStick) {

			frontLeft.setDriveSpeed(0);
			frontRight.setDriveSpeed(0);
			backLeft.setDriveSpeed(0);
			backRight.setDriveSpeed(0);

			frontLeft.setSteerSpeed(0);
			frontRight.setSteerSpeed(0);
			backLeft.setSteerSpeed(0);
			backRight.setSteerSpeed(0);

		} else {

			/*
			 * Set each swerve module, scaling the drive speeds by the maximum speed
			 */
			frontLeft.setSwerve(angleFL, speedFL / maxSpeed, this._driveCorrect);
			frontRight.setSwerve(angleFR, speedFR / maxSpeed, this._driveCorrect);
			backLeft.setSwerve(angleBL, speedBL / maxSpeed, this._driveCorrect);
			backRight.setSwerve(angleBR, speedBR / maxSpeed, this._driveCorrect);
		}

		// getSteerEncoderVal();
	}

	private double speed(double val1, double val2) {
		return Math.hypot(val1, val2);
	}

	private double angle(double val1, double val2) {
		return Math.toDegrees(Math.atan2(val1, val2));
	}

	public double[] getDriveEncoders() {
		double[] values = new double[] {
			frontLeft.getDriveEncoder(),
			frontRight.getDriveEncoder(),
			backLeft.getDriveEncoder(),
			backRight.getDriveEncoder()
		};

		return values;
	}

	public double getDriveEncoderAvg() {
		double driveFL = Math.abs(frontLeft.getDriveEncoder());
		double driveFR = Math.abs(frontRight.getDriveEncoder());
		double driveBL = Math.abs(backLeft.getDriveEncoder());
		double driveBR = Math.abs(backRight.getDriveEncoder());
		return (driveFL + driveFR + driveBL + driveBR) / 4.0;
	}

	public void setDriveEncodersPosition(double position) {
		frontLeft.setDriveEncoder(position);
		frontRight.setDriveEncoder(position);
		backLeft.setDriveEncoder(position);
		backRight.setDriveEncoder(position);
	}

	public void getSteerEncoderVal() {
		SmartDashboard.putNumber("angleLF", frontLeft.rawEncoderPosition());
		SmartDashboard.putNumber("angleRF", frontRight.rawEncoderPosition());
		SmartDashboard.putNumber("angleLB", backLeft.rawEncoderPosition());
		SmartDashboard.putNumber("angleRB", backRight.rawEncoderPosition());
	}

	public SwerveModulePosition[] getPositions() {
		return new SwerveModulePosition[] {
			frontLeft.getPosition(),
			frontRight.getPosition(),
			backLeft.getPosition(),
			backRight.getPosition()
		};
	}

	@Override()
	public void periodic() {

		/*
		 * The state of the robot gyro and individual swerve modules are
		 * sent to odometer on each cycle of the program.
		 */
		if(this._navXGyro != null) {
			odometer.update(this._navXGyro.getRotation2d(), getPositions());
			SmartDashboard.putNumber("Robot Heading", this._navXGyro.getHeading());
		} else if (this._pigeonGyro != null) {
			odometer.update(this._pigeonGyro.getRotation2d(), getPositions());
			SmartDashboard.putNumber("Yaw Value", this._pigeonGyro.getGyroYawValue());
			SmartDashboard.putNumber("Robot Heading", this._pigeonGyro.getHeading());
		}

		SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
		SmartDashboard.putNumber("Robot DistanceX", odometer.getPoseMeters().getX());
		SmartDashboard.putNumber("Robot DistanceY", odometer.getPoseMeters().getY());
	}

	public void stopModules() {
		frontLeft.stop();
		frontRight.stop();
		backLeft.stop();
		backRight.stop();
	}

	// Set Drive mode for balance Auto
	public void setDrivesMode(IdleMode idleMode) {
		frontLeft.setDriveMode(idleMode);
		frontRight.setDriveMode(idleMode);
		backLeft.setDriveMode(idleMode);
		backRight.setDriveMode(idleMode);
	}

	public void setDriveModeCoast() {
		setDrivesMode(IdleMode.kCoast);
		isCoastMode = true;
	}

	public void setDriveModeBrake() {
		setDrivesMode(IdleMode.kBrake);
		isCoastMode = false;
	}

	public void disableRamping() {
		frontLeft.driveMotorRamp(false);
		frontRight.driveMotorRamp(false);
		backLeft.driveMotorRamp(false);
		backRight.driveMotorRamp(false);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
			SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
					DriveConstants.ModuleConstants.kPhysicalMaxSpeedMetersPerSecond);
			frontLeft.setDesiredState(desiredStates[0]);
			frontRight.setDesiredState(desiredStates[1]);
			backLeft.setDesiredState(desiredStates[2]);
			backRight.setDesiredState(desiredStates[3]);
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		states[0] = frontLeft.getState();
		states[1] = frontRight.getState();
		states[2] = backLeft.getState();
		states[3] = backRight.getState();
		return states;
	}


	private boolean isCoastMode = false;
	public boolean toggleMode() {
		return isCoastMode;
	}
}