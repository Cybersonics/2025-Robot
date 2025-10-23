package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.Arrays;
import java.util.Collections;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import frc.robot.subsystems.Camera;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class Drive extends SubsystemBase {

	private static Drive instance;

	public static final double kMaxSpeed = 3.0; // 3 meters per second
	public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

	private static SwerveModule frontLeft;
	private static SwerveModule backLeft;
	private static SwerveModule frontRight;
	private static SwerveModule backRight;

	public double heading;
	public double angle;

	public static final double length_with_bumpers = .9271;
	public static final Pose2d in_front_of_tag_18 = new Pose2d(3.6576-length_with_bumpers/2,4.0259,Rotation2d.fromDegrees(0));

	public static final double OMEGA_SCALE = 1.0 / 30.0;

	private final boolean invertDrive = false;//true;
	private final boolean invertSteer = true; //true;
	private NavXGyro _navXGyro;
	private PigeonGyro _pigeonGyro;
	private boolean _driveCorrect= false;

	//private final SwerveDriveOdometry odometer;

	private SwerveDrivePoseEstimator robotPoseEstimate;

	public Camera _camera;
	private PhotonCamera _photonCamera;


	private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
	//private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
	public PhotonPoseEstimator photonPoseEstimator;
    public Transform3d robotToCam;

	private Matrix<N3, N1> curStdDevs;

	        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

	/*
	 * Set up the drive by passing in the gyro and then configuring the individual
	 * swerve modules.
	 * Note the order that the modules are in. Be consistant with the order in the
	 * odometry.
	 */
	private Drive(PigeonGyro gyro, Camera camera) {
	//private Drive(PigeonGyro gyro) {

		this._pigeonGyro = gyro;
		this._camera = camera;
		this._photonCamera = camera.getPhotonCamera();

		frontLeft = new SwerveModule(DriveConstants.FrontLeftSteer, DriveConstants.FrontLeftDrive, invertDrive,
				invertSteer);

		frontRight = new SwerveModule(DriveConstants.FrontRightSteer, DriveConstants.FrontRightDrive, invertDrive,
		 		invertSteer);

		backLeft = new SwerveModule(DriveConstants.BackLeftSteer, DriveConstants.BackLeftDrive, invertDrive,
		 		invertSteer);

		backRight = new SwerveModule(DriveConstants.BackRightSteer, DriveConstants.BackRightDrive, invertDrive,
		 		invertSteer);

		// odometer = new SwerveDriveOdometry(DriveConstants.FrameConstants.kDriveKinematics,
		//  		this._pigeonGyro.getGyroRotation2D(), getPositions());

		robotPoseEstimate = new SwerveDrivePoseEstimator(DriveConstants.FrameConstants.kDriveKinematics, 
			this._pigeonGyro.getGyroRotation2D(), getPositions(), Pose2d.kZero,
			VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
			VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));


		RobotConfig robotConfig;
		try{
			robotConfig = RobotConfig.fromGUISettings();
			
			// Configure AutoBuilder
			AutoBuilder.configure(
					this::getPose,
					//this::getRobotPose,
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

			robotToCam = new Transform3d(new Translation3d(
				Constants.CameraConstants.cameraPositionX,
				Constants.CameraConstants.cameraPositionY,
				Constants.CameraConstants.cameraPositionZ), 
				new Rotation3d(
				Constants.CameraConstants.cameraPositionRoll,
				Constants.CameraConstants.cameraPositionPitch,
				Constants.CameraConstants.cameraPositionYaw));
	
			// Construct PhotonPoseEstimator
			photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				robotToCam);
			
			photonPoseEstimator.setMultiTagFallbackStrategy((PoseStrategy.LOWEST_AMBIGUITY));
		}
	}

	/*
	 * Set up the drive by passing in the gyro and then configuring the individual
	 * swerve modules.
	 * Note the order that the modules are in. Be consistant with the order in the
	 * odometry.
	 */
	//private Drive(NavXGyro gyro) {
	private Drive(NavXGyro gyro, Camera camera) {

		this._navXGyro = gyro;
		this._camera = camera;
		this._photonCamera = camera.getPhotonCamera();

		frontLeft = new SwerveModule(DriveConstants.FrontLeftSteer, DriveConstants.FrontLeftDrive, invertDrive,
				invertSteer);

		frontRight = new SwerveModule(DriveConstants.FrontRightSteer, DriveConstants.FrontRightDrive, invertDrive,
		 		invertSteer);

		backLeft = new SwerveModule(DriveConstants.BackLeftSteer, DriveConstants.BackLeftDrive, invertDrive,
		 		invertSteer);

		backRight = new SwerveModule(DriveConstants.BackRightSteer, DriveConstants.BackRightDrive, invertDrive,
		 		invertSteer);

		// odometer = new SwerveDriveOdometry(DriveConstants.FrameConstants.kDriveKinematics,
		//  		this._navXGyro.getGyroRotation2D(), getPositions());

		resetOdometry(in_front_of_tag_18);
		
		robotPoseEstimate = new SwerveDrivePoseEstimator(DriveConstants.FrameConstants.kDriveKinematics, 
			this._navXGyro.getGyroRotation2D(), getPositions(), in_front_of_tag_18,
			VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
			VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

		RobotConfig robotConfig;
		try{
			robotConfig = RobotConfig.fromGUISettings();
			
			// Configure AutoBuilder
			AutoBuilder.configure(
					this::getPose,
					//this::getRobotPose,
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
		robotToCam = new Transform3d(new Translation3d(
            Constants.CameraConstants.cameraPositionX,
            Constants.CameraConstants.cameraPositionY,
            Constants.CameraConstants.cameraPositionZ), 
            new Rotation3d(
            Constants.CameraConstants.cameraPositionRoll,
            Constants.CameraConstants.cameraPositionPitch,
            Constants.CameraConstants.cameraPositionYaw));

    	// Construct PhotonPoseEstimator
		photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			robotToCam);
		
		photonPoseEstimator.setMultiTagFallbackStrategy((PoseStrategy.LOWEST_AMBIGUITY));
	}

	// Public Methods

	public Pose2d getPose() {
		return robotPoseEstimate.getEstimatedPosition();
	 	// return odometer.getPoseMeters();
	}

	//public Pose2d getRobotPose(){
	//	return robotPoseEstimate.getEstimatedPosition();
	//}

	public void resetPose(Pose2d pose) {
		if(this._navXGyro != null) {
			//odometer.resetPosition(this._navXGyro.getGyroRotation2D(), getPositions(), pose);
			robotPoseEstimate.resetPosition(this._navXGyro.getRotation2d(),getPositions(), pose);
		} else if (this._pigeonGyro != null) {
			//odometer.resetPosition(this._pigeonGyro.getGyroRotation2D(), getPositions(), pose);
			robotPoseEstimate.resetPosition(this._pigeonGyro.getRotation2d(),getPositions(), pose);
		}
	}

	public ChassisSpeeds getSpeeds() {
		return DriveConstants.FrameConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
	}

	public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
		driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
		//driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getRobotPose().getRotation()));
	}

	public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {	
		ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

		SwerveModuleState[] targetStates = DriveConstants.FrameConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
		setModuleStates(targetStates);
	}

	public void resetOdometry(Pose2d pose) {
		if(this._navXGyro != null) {
			//odometer.resetPosition(this._navXGyro.getRotation2d(), getPositions(), pose);
			robotPoseEstimate.resetPosition(this._navXGyro.getRotation2d(),getPositions(), pose);
		} else if (this._pigeonGyro != null) {
			//odometer.resetPosition(this._pigeonGyro.getRotation2d(), getPositions(), pose);
			robotPoseEstimate.resetPosition(this._pigeonGyro.getRotation2d(),getPositions(), pose);
		}
	}

	// public void resetOdometryForState(PathPlannerState state) {
	// state = PathPlannerTrajectory.transformStateForAlliance(state, DriverStation.getAlliance());
	// Pose2d pose = new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
	// odometer.resetPosition(this._gyro.getRotation2d(), getPositions(), pose);
	// }

	//public static Drive getInstance(NavXGyro gyro) {
	public static Drive getInstance(NavXGyro gyro, Camera camera) {
		if (instance == null) {
			instance = new Drive(gyro, camera);
			//instance = new Drive(gyro);
		}
		return instance;
	}

	//public static Drive getInstance(PigeonGyro gyro) {
	public static Drive getInstance(PigeonGyro gyro, Camera camera) {
		if (instance == null) {
			instance = new Drive(gyro, camera);
			//instance = new Drive(gyro);
		}
		return instance;
	}

	public void processInput(double forward, double strafe, double omega, boolean deadStick) {

		this._driveCorrect = false;

		double omegaL2 = omega * (DriveConstants.FrameConstants.WHEEL_BASE_LENGTH / 2.0);
		double omegaW2 = omega * (DriveConstants.FrameConstants.WHEEL_BASE_WIDTH / 2.0);

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
		SmartDashboard.putNumber("angleLF", frontLeft.getTurningPosition());
		SmartDashboard.putNumber("angleRF", frontRight.getTurningPosition());
		SmartDashboard.putNumber("angleLB", backLeft.getTurningPosition());
		SmartDashboard.putNumber("angleRB", backRight.getTurningPosition());
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

		int _aprilTagID=-1;
	 	PhotonTrackedTarget _bestTarget;

		/*
		 * The state of the robot gyro and individual swerve modules are
		 * sent to odometer on each cycle of the program.
		 */
		if(this._navXGyro != null) {
			//odometer.update(this._navXGyro.getRotation2d(), getPositions());
			//SmartDashboard.putNumber("Robot Heading", this._navXGyro.getHeading());

			robotPoseEstimate.update(this._navXGyro.getRotation2d(),getPositions());
			SmartDashboard.putNumber("Yaw Value", -this._navXGyro.getGyroAngle());
			//SmartDashboard.putNumber("Robot Heading", this._navXGyro.getHeading());

		} else if (this._pigeonGyro != null) {
			//odometer.update(this._pigeonGyro.getRotation2d(), getPositions());
			//SmartDashboard.putNumber("Yaw Value", this._pigeonGyro.getGyroYawValue());
			//SmartDashboard.putNumber("Robot Heading", this._pigeonGyro.getHeading());

			robotPoseEstimate.update(this._pigeonGyro.getRotation2d(),getPositions());
		}
		
		
		Optional<EstimatedRobotPose> testVisionEst = Optional.empty();
		
		// Get latest frame from Camera 
		var latestResults = _photonCamera.getAllUnreadResults();
		if(!latestResults.isEmpty()){
			double vision_time_stamp = latestResults.get(latestResults.size()-1).getTimestampSeconds();
            var result = latestResults.get(latestResults.size() - 1);

            if (result.hasTargets()) {
				_bestTarget = null;
				double bestX = 9999;
				for (PhotonTrackedTarget target : result.getTargets()) {
					if(target.getBestCameraToTarget().getX() < bestX) {
						_bestTarget = target;
						bestX = target.getBestCameraToTarget().getX();
					}
				}
				//List<Pose3d> testResult = latestResults.sort((result) => result.targets.sort(null););

				// At least one Apriltag was seen by the camera

				_aprilTagID = _bestTarget.getFiducialId();
				Optional<Pose3d> field_to_april_tag = aprilTagFieldLayout.getTagPose(_aprilTagID);
				if(field_to_april_tag.isPresent()){
					Pose3d tagPose = field_to_april_tag.get();
					Transform3d fieldToTag = new Transform3d(tagPose.getTranslation(),tagPose.getRotation());
					Transform3d bestCameraToTag = _bestTarget.getBestCameraToTarget();
					Transform3d bestTagToCamera = bestCameraToTag.inverse();
					Transform3d fieldToCamera = fieldToTag.plus(bestTagToCamera);
					Transform3d estimated_robot_position_kal_3d = fieldToCamera.plus(robotToCam.inverse());
					Pose3d estimated_robot_position_kal = new Pose3d(estimated_robot_position_kal_3d.getTranslation(), estimated_robot_position_kal_3d.getRotation());
					Pose2d estimated_robot_position_kal_2d = estimated_robot_position_kal.toPose2d();
					Pose3d camera_pose = new Pose3d(fieldToCamera.getTranslation(),fieldToCamera.getRotation());
					Pose3d robot_pose = new Pose3d(robotPoseEstimate.getEstimatedPosition());
					
					Transform3d robot_to_camera_transform = new Transform3d(robot_pose,camera_pose);

					//SmartDashboard.putNumber("poseod X", odometer.getPoseMeters().getX());
					//SmartDashboard.putNumber("poseod y", odometer.getPoseMeters().getY());
					//SmartDashboard.putNumber("poseod theta", odometer.getPoseMeters().getRotation().getDegrees());

					SmartDashboard.putNumber("poseest X", robotPoseEstimate.getEstimatedPosition().getX());
					SmartDashboard.putNumber("poseest y", robotPoseEstimate.getEstimatedPosition().getY());
					SmartDashboard.putNumber("poseest theta", robotPoseEstimate.getEstimatedPosition().getRotation().getDegrees());
					SmartDashboard.putNumber("r2c X", robot_to_camera_transform.getX());
					SmartDashboard.putNumber("r2c y", robot_to_camera_transform.getY());
					SmartDashboard.putNumber("r2c z", robot_to_camera_transform.getZ());
					SmartDashboard.putNumber("r2c aboutx", Units.radiansToDegrees(robot_to_camera_transform.getRotation().getX()));
					SmartDashboard.putNumber("r2c abouty", Units.radiansToDegrees(robot_to_camera_transform.getRotation().getY()));
					SmartDashboard.putNumber("r2c aboutz", Units.radiansToDegrees(robot_to_camera_transform.getRotation().getZ()));
				
					testVisionEst = photonPoseEstimator.update(result);
					if( testVisionEst.isPresent()){
						
						robotPoseEstimate.addVisionMeasurement(estimated_robot_position_kal_2d,vision_time_stamp);
					}
				}
				// double poseAmbiguity = _bestTarget.getPoseAmbiguity();
				// SmartDashboard.putNumber("Cam TagID", _aprilTagID);
				// SmartDashboard.putNumber("Cam Yaw", _bestTarget.yaw);
				// SmartDashboard.putNumber("Cam X", (Units.metersToInches(bestCameraToTarget.getX()) - 2.0));
				// SmartDashboard.putNumber("Cam Y", Units.metersToInches(bestCameraToTarget.getY()));
				// SmartDashboard.putNumber("Cam Z", Units.metersToInches(bestCameraToTarget.getZ()));
				// SmartDashboard.putNumber("Cam Ambiguity", poseAmbiguity);
			

				// testVisionEst.ifPresent(p->{
				// 	SmartDashboard.putNumber("X-ValCam", Units.metersToInches(p.estimatedPose.getX()));
				// 	SmartDashboard.putNumber("Y-ValCam", Units.metersToInches(p.estimatedPose.getY()));
				// 	SmartDashboard.putNumber("Z-ValCam", Units.metersToInches(p.estimatedPose.getZ()));
				// 	SmartDashboard.putNumber("Yaw-ValCam", Math.toDegrees(p.estimatedPose.getRotation().getZ()));
				// });
			}

       	}
		
      	//if (_aprilTagID>-1){
         	  //Pose3d target = _aprilTag._fieldLayout.getTagPose(_aprilTagID).get();
			//Pose3d target = aprilTagFieldLayout.getTagPose(_aprilTagID).get();




			// Optional<EstimatedRobotPose> visionEst = Optional.empty();
        	// for (var change : _photonCamera.getAllUnreadResults()) {
            // 	visionEst = photonPoseEstimator.update(change);
            // 	updateEstimationStdDevs(visionEst, change.getTargets());
			// 	visionEst.ifPresent(
            //         est -> {
            //             // Change our trust in the measurement based on the tags we can see
            //             var estStdDevs = getEstimationStdDevs();

            //             estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            //         });
        	// }

			  //Pose2d test2d = photonPoseEstimator.getReferencePose().toPose2d();
				
			//Pose2d visionMeasurement2d = target.toPose2d();

    		  // Apply vision measurements. For simulation purposes only, we don't input a latency delay -- on
   			  // a real robot, this must be calculated based either on known latency or timestamps

   			  //robotPoseEstimate.addVisionMeasurement(visionMeasurement2d, Timer.getFPGATimestamp());
			  //robotPoseEstimate.addVisionMeasurement(test2d, Timer.getFPGATimestamp());
			
	 	//}


		//SmartDashboard.putString("Robot Location", getRobotPose().getTranslation().toString());
		//SmartDashboard.putNumber("Robot DistanceX", robotPoseEstimate.getEstimatedPosition().getX());
		//SmartDashboard.putNumber("Robot DistanceY", robotPoseEstimate.getEstimatedPosition().getY());

		// if (this._camera.hasTargets()){
		// 	SmartDashboard.putNumber("Cam TagID", this._camera.getTargetId());
		// 	SmartDashboard.putNumber("Cam Yaw", this._camera.getYawVal());
		// 	SmartDashboard.putNumber("Cam X", (Units.metersToInches(this._camera.getXVal()) - 2.0));
		// 	SmartDashboard.putNumber("Cam Y", Units.metersToInches(this._camera.getYVal()));
		// 	SmartDashboard.putNumber("Cam Z", Units.metersToInches(this._camera.getZVal()));
		// 	SmartDashboard.putNumber("Cam Ambiguity", this._camera.getPoseAmbiguityVal());
				
		// 	SmartDashboard.putNumber("X-ValCam", Units.metersToInches(this._camera.getEstPoseXVal()));
		// 	SmartDashboard.putNumber("Y-ValCam", Units.metersToInches(this._camera.getEstPoseYVal()));
		// 	SmartDashboard.putNumber("Z-ValCam", Units.metersToInches(this._camera.getEstPoseZVal()));
		// 	SmartDashboard.putNumber("Yaw-ValCam", Math.toDegrees(this._camera.getEstPoseYawVal()));

		// 	//SmartDashboard.putNumber("Robot DistanceX", odometer.getPoseMeters().getX());
		// 	//SmartDashboard.putNumber("Robot DistanceY", odometer.getPoseMeters().getY());
		// }
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
					DriveConstants.FrameConstants.kPhysicalMaxSpeedMetersPerSecond / 2);
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
 
	// private void updateEstimationStdDevs(
    //         Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    //     if (estimatedPose.isEmpty()) {
    //         // No pose input. Default to single-tag std devs
    //         curStdDevs = kSingleTagStdDevs;

    //     } else {
    //         // Pose present. Start running Heuristic
    //         var estStdDevs = kSingleTagStdDevs;
    //         int numTags = 0;
    //         double avgDist = 0;

    //         // Precalculation - see how many tags we found, and calculate an average-distance metric
    //         for (var tgt : targets) {
    //             var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
    //             if (tagPose.isEmpty()) continue;
    //             numTags++;
    //             avgDist +=
    //                     tagPose
    //                             .get()
    //                             .toPose2d()
    //                             .getTranslation()
    //                             .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
    //         }

    //         if (numTags == 0) {
    //             // No tags visible. Default to single-tag std devs
    //             curStdDevs = kSingleTagStdDevs;
    //         } else {
    //             // One or more tags visible, run the full heuristic.
    //             avgDist /= numTags;
    //             // Decrease std devs if multiple targets are visible
    //             if (numTags > 1) estStdDevs = kMultiTagStdDevs;
    //             // Increase std devs based on (average) distance
    //             if (numTags == 1 && avgDist > 4)
    //                 estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    //             else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    //             curStdDevs = estStdDevs;
    //         }
    //     }
    // }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    // @FunctionalInterface
    // public static interface EstimateConsumer {
    //     public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    // }

}