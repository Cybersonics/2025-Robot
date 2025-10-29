package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants.FrameConstants;


import static edu.wpi.first.units.Units.*;

public class Constants {
    public static class OperatorConstants {
        public static final int DriveController = 0;
        public static final int OpController = 1;
    }

    public static class AutoConstants {
        public static final double kPhysicalMaxSpeedMetersPerSecond = FrameConstants.kPhysicalMaxSpeedMetersPerSecond;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

        public static final double kMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond; // / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond; // / 5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.5; //3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;
        public static final double kPXController = 5;
        public static final double kPYController = 5;
        public static final double kPThetaController = 5;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
        public static final double kGoToPointLinearP = 0;
        public static final double kGoToPointLinearF = 0.5;
        public static final double kGoToPointAngularP = 0;
        public static final double kGoToPointAngularF = 0;

        public static final double maxTrajectoryOverrunSeconds = 3;
        public static final double kMaxDistanceMetersError = 0.1;
        public static final double kMaxAngleDegreesError = 5;
    }

    public static class DriveConstants {
        public static final int PigeonCANId = 5;

        public static final int FrontLeftSteer = 22;//20;
        public static final int FrontLeftDrive = 12;//10;
        public static final int FrontLeftEncoderOffset = 0;

        public static final int FrontRightSteer = 21;//23;
        public static final int FrontRightDrive = 11;//13;
        public static final int FrontRightEncoderOffset = 0;

        public static final int BackLeftSteer = 23;//21;
        public static final int BackLeftDrive = 13;//11;
        public static final int BackLeftEncoderOffset = 0;

        public static final int BackRightSteer = 20;//22;
        public static final int BackRightDrive = 10;//12;
        public static final int BackRightEncoderOffset = 0;

        public static final class ModuleConstants {
            public static final double kWheelDiameterMeters = Units.inchesToMeters(4.1); // 4
            public static final double kDriveMotorGearRatio = 1 / 6.429;

            // Invert the turning encoder if necessary. If pivots snap back and forth when setting up
            // inversion may be needed as the encoder can be reading in the opposite direction 
            // as the pivots are going. 
            public static final boolean kTurningEncoderInverted = false;


            // Calculations required for driving motor conversion factors and feed forward
            public static final double kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60;
            public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
            public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kDriveMotorGearRatio)
             * kWheelCircumferenceMeters;

            public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            * kDriveMotorGearRatio; // meters

            public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            * kDriveMotorGearRatio) / 60.0; // meters per second

            public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
            public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
        
            public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
            public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians
        
            public static final double kDrivingP = 0.04;
            public static final double kDrivingI = 0;
            public static final double kDrivingD = 0;
            public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
            public static final double kDrivingMinOutput = -1;
            public static final double kDrivingMaxOutput = 1;
        
            public static final double kTurningP = 0.8;
            public static final double kTurningI = 0;
            public static final double kTurningD = 0.005;
            public static final double kTurningFF = 0;
            public static final double kTurningMinOutput = -1;
            public static final double kTurningMaxOutput = 1;

            
            // Theoretical free speed (m/s) at 12 V applied output;
            // This needs to be tuned to your individual robot
            public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(FrameConstants.kPhysicalMaxSpeedMetersPerSecond);

        }

        public static final class FrameConstants {
            // Distance between right and left wheels
            public static final double WHEEL_BASE_WIDTH = 22;
            public static final double kTrackWidth = Units.inchesToMeters(WHEEL_BASE_WIDTH);

            // Distance between front and back wheels
            public static final double WHEEL_BASE_LENGTH = 22;
            public static final double kWheelBase = Units.inchesToMeters(WHEEL_BASE_LENGTH);

            public static final Translation2d flModuleOffset = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
            public static final Translation2d frModuleOffset = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
            public static final Translation2d blModuleOffset = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
            public static final Translation2d brModuleOffset = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);

            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                flModuleOffset,
                frModuleOffset,
                blModuleOffset,
                brModuleOffset
            );
                    
            public static final double kPhysicalMaxSpeedMetersPerSecond = 4.69; //6;
            public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
        }

        public static final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
                new PIDConstants(7, 0, 0.01), // Translation constants P=5.0, I=0, D=0 kP=2.3
                new PIDConstants(7.5, 0, 0)); // Rotation constants P=5.0, I=0, D=0  kP=.68 2.8 .65
    }

    public static class ElevatorConstants {
        public final static int leftMotorCANId = 30;
        public final static int rightMotorCANId = 31;
        public final static int stringPotChannel = 0;

        // Elevator constants
	    public static final double ELEVATOR_POSITION_TOLERANCE = 20;//0.2;
	    public static final double ELEVATOR_CONVERSION_FACTOR = 25;
	    public static final double ELEVATOR_ROLLER_RAIDUS = 3;
        
	    public static final double ELEVATOR_MAX_VELO = 7500;//6500//1000; //3000;
	    public static final double ELEVATOR_MAX_ACCELLERATION = 7500;//7500//1000; //5000;
	    public static final double ELEVATOR_P = 0.027; //0.025// 0.02; //.012; //0.073;
	    public static final double ELEVATOR_I = 0.0;
	    public static final double ELEVATOR_D = 0.0005; //0.0026;//0.005;
	    public static final int ELEVATOR_CURRENT_LIMIT = 40;
  	    public static final double ELEVATOR_MANUAL_POWER = 0.1;
  	    // public static final double L1_HEIGHT = 0;
  	    // public static final double L2_HEIGHT = 10.833;
  	    // public static final double L3_HEIGHT = 26.85;
  	    // public static final double L4_HEIGHT = 52.16;
    }

    public static class AlgeaConstants {
        public final static int algeaMotorCANId = 42;
    }

    public static class CoralConstants {
        public final static int leftCoralMotorCANId = 41;
        public final static int rightCoralMotorCANId = 40;

        public final static int coralFeedSesorId = 0;
        public final static int coralTripSensorId = 1;
    }

    public static final class VortexMotorConstants {
        public static final double kFreeSpeedRpm = 6784;
    }

    public static final class PneumaticConstants {
        public static final int compressorInput = 1;
        
        public static final int algeaSolenoid = 15;
        public static final int climberSolenoid = 14;
    }

    public static final class ClimberConstants {
        public static final int climberCANId = 50;

        public static final double kSpoolDiameterMeters = Units.inchesToMeters(2); // 4
        public static final double kClimberMotorGearRatio = 1 / 45;
        public static final double kClimberEncoderRot2Meter = kClimberMotorGearRatio * Math.PI * kSpoolDiameterMeters;
    }

    // public static final class CameraConstants {
    //     public static final String CameraName = "Arducam_OV9281_USB_Camera"; 
    //     public static final int AprilTagPipeline = 0;

    //     public static final double cameraPositionX = Units.inchesToMeters(-1.5);//-2.0//-2.5//-3//20.375
    //     public static final double cameraPositionY = Units.inchesToMeters(0);//0
    //     public static final double cameraPositionZ = Units.inchesToMeters(14.25);//14.5

    //     public static final double cameraPositionRoll = Units.degreesToRadians(0);
    //     public static final double cameraPositionPitch = Units.degreesToRadians(0);
    //     public static final double cameraPositionYaw = Units.degreesToRadians(0);

        

    // }

   public static final class VisionConstants {
    // For the robot to camera transforms, the robot coordinates are used.
    // The input to the transform is the distance, in meters, from the center
    // of the robot to the location where the camera is mounted.  Check the FRC
    // site for additional information on the robot coordinate system.
    // Looking at the robot from above with the front on top:
    // X is positive towards the front of the robot and negative towards the rear
    // Y is positive towards the left and negative towards the right
    // Z is positive away from the ground and negative towards the ground
    // For the rotations, the angle is in radians as follows:
    // Roll is rotation around the X axis (not used) 
    // Pitch is rotation around the Y axis (not used)
    // Yaw is rotation around the Z axis and it is positive counter clockwise and
    // negative clockwise
    public static final Transform3d BOT_TO_LEFT_CAM = new Transform3d(
     new Translation3d(
        Units.inchesToMeters(-10.5), //10.0 Forward/back/ X
        Units.inchesToMeters(-10.6875), //11,125 Left/Right/ Y
        Units.inchesToMeters(13.25)),//13.25 Up/down/ Z
      new Rotation3d(Math.toRadians(0), Math.toRadians(5.0), Math.toRadians(151.5))//152.5  151  153.3    0,5,-26.7
      ); 

    public static final Transform3d BOT_TO_RIGHT_CAM = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(-6.125), //6.125 Forward/back/ X
        Units.inchesToMeters(7.5), //-7.5 Left/Right/ Y
        Units.inchesToMeters(10.75)), //10.75 Up/Down/ Z
      new Rotation3d(Math.toRadians(0), 0, Math.toRadians(175.5))//0,0,4.5
    );

    public static final double AMBIGUITY_THRESHOLD = 0.3;
    public static final double minAprilTagSize = 0.8; // This is the minimum size for an AprilTag to be considered good to work with.
    public static final double midRangeAprilTagSize = 0.4;
    public static final double minAprilTagAngleToRobot = Math.PI/4;  // This is the minimum angle delta bewteen robot and AprilTag for an AprilTag to be considered good to work with.

    public static final Pose2d noPose = new Pose2d();
    public static final PhotonPipelineResult noPipeResult = new PhotonPipelineResult();
  }


  public static final class AutoScore{
    // Constants to select which coral branch to score, the left or the right, from the perspective of the driver.
    public static final boolean RIGHT  = true;  // Scoring position is to the right of the driver's perspective.
    public static final boolean LEFT = false;   // Scoring position is to the left of the driver's perspective.

    // Distance from scoring position (referenced to the center of the robot) to the AprilTag.  This distance is
    // measured along a line perpendicular to the AprilTag.
    public static final double scorePosDistance = 0.508;//0.508;//0.47;  // Units are meters. This distance should be adjected depending on robot length.
    public static final double algaePosDistance = 0.47;  // Units are meters.  This is the distance from the coral reef to the center of the robot.
    public static final double scoreL1Distance = 0.40;   // Units are meters.  

    // Distance from the center of the AprilTag to scoring position to either the left or right or the AprilTag.
    // The scoring positions are symetric to the left or right (i.e., same distance to the right or left or the
    // AprilTag)
    public static final double scorePosOffset_LEFT = 0.4128;//0.3874;//0.165; // In meters.  Scoring position distance to the LEFT of the AprilTag for LEFT branch scoring.
    public static final double scorePosOffset_RIGHT = -0.0762;//-0.1715;//0.4382;//03366;//0.08255;//0.165; // In meters.  Scoring position distance to the RIGHT of the AprilTag for RIGHT branch scoring.

    public static final double scoreL1Offset_LEFT = 0.5158; // In meters.  Scoring position distance to the LEFT of the AprilTag for LEFT branch scoring.
    public static final double scoreL1Offset_RIGHT = 0.5158; // In meters.  Scoring position distance to the RIGHT of the AprilTag for RIGHT branch scoring.

    public static final double scoreL1Oangle_LEFT = 0; // In degrees.  This is the end robot angle with respect to the reef.  NOTE: adjusting this angle may require adjusting the distance.
    public static final double scoreL1Oangle_RIGHT = 0; // In degrees.  This is the end robot angle with respect to the reef. NOTE: adjusting this angle may require adjusting the distance.

    // Maximum time allowed for executing the GoToScoringPosition command
    public static final double maxTime = 2.0;  // Time out in seconds.
    public static final double distToShootCoral = 0.2; // This is the distance from the scoring position to shoot the coral for auton.
  }

  public static enum ScoringPresets{
    STOW,
    ALGAE_LOW,
    ALGAE_HIGH,
    L1,
    L2,
    L3,
    L4
  }

  
}