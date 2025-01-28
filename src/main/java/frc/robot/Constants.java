package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.utility.shuffleBoardDrive;
import frc.robot.utility.AprilTag;


public class Constants {
    public static class OperatorConstants {
        public static final int LeftStick = 0;
        public static final int RightStick = 1;
        public static final int OpController = 2;
        public static final int xboxDriveController = 1;
    }

    public static class AutoConstants {
        public static final double kPhysicalMaxSpeedMetersPerSecond = 6;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

        public static final double kMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
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
        public static final int FrontLeftSteer = 20;
        public static final int FrontLeftDrive = 10;
        public static final int FrontLeftEncoderOffset = 0;

        public static final int FrontRightSteer = 23;
        public static final int FrontRightDrive = 13;
        public static final int FrontRightEncoderOffset = 0;

        public static final int BackLeftSteer = 21;
        public static final int BackLeftDrive = 11;
        public static final int BackLeftEncoderOffset = 0;

        public static final int BackRightSteer = 22;
        public static final int BackRightDrive = 12;
        public static final int BackRightEncoderOffset = 0;

        public static final shuffleBoardDrive frontLeft = new shuffleBoardDrive("LF Set Angle", 0, 0);
        public static final shuffleBoardDrive backLeft = new shuffleBoardDrive("LB Set Angle", 0, 1);

        public static final shuffleBoardDrive frontRight = new shuffleBoardDrive("RF Set Angle", 4, 0);
        public static final shuffleBoardDrive backRight = new shuffleBoardDrive("RBack Set Angle", 4, 1);

        public static final class ModuleConstants {
            public static final double kWheelDiameterMeters = Units.inchesToMeters(4.1); // 4
            public static final double kDriveMotorGearRatio = 1 / 6.429;
            public static final double kTurningMotorGearRatio = 1 / 1024;
            public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
            public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
            public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
            public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

            public static final double maxSpeed = 6.5; // M/S

            public static final double kPTurning = 0.5;

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

        }

        public static final class FrameConstants {
            // Distance between right and left wheels
            public static final double WHEEL_BASE_WIDTH = 21;
            public static final double kTrackWidth = Units.inchesToMeters(WHEEL_BASE_WIDTH);

            // Distance between front and back wheels
            public static final double WHEEL_BASE_LENGTH = 17.25;
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
                    
            public static final double kPhysicalMaxSpeedMetersPerSecond = 6;
            public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
        }

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(2.3, 0, 0.01), // Translation constants P=5.0, I=0, D=0
                new PIDConstants(2.0, 0, 0), // Rotation constants P=5.0, I=0, D=0
                ModuleConstants.maxSpeed,
                FrameConstants.flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module)
                new ReplanningConfig());
    }

    public static final class IntakeConstants {
        public static final int intakeMotor = 30;
        public static final int feederMotor = 34;

        public static final int topNoteTripInput = 0;       
        public static final int bottomNoteTripInput = 1;

    }

    public static final class LauncherConstants {
        public static final int launcherTopMotor = 32;
        public static final int launcherBottomMotor = 31;
        public static final int feederMotor = 33;

        public static final double topFarShotRPM = 5300; // 5255; // avg drop down
        public static final double bottomFarShotRPM = 5440; // 5051; // avg drop down
        
        public static final double topShortShotRPM = 5300; // 5255; // avg drop down
        public static final double bottomShortShotRPM = 5440; // 5051; // avg drop down

        public static final double topFarLobRPM = 5000; // ;
        public static final double bottomFarLobRPM = 5100; // ;

        public static final double topSourceLobRPM = 3900;
        public static final double bottomSourceLobRPM = 3900;

        public static final double topAmpShotRPM = 4000;
        public static final double bottomAmpShotRPM = 4000;

        public static final double topTrapShotRPM = 4800;
        public static final double bottomTrapShotRPM = 4800;
    }

    public static final class ShuffleBoardConstants {
        private static ShuffleboardTab driveTab = Shuffleboard.getTab("Drive Tab");
    }

    public static class shuffleBoardDrive {
        public String drivePosition;
        public int colPos;
        public int rowPos;
    
        public shuffleBoardDrive(String drivePosition, int colPos, int rowPos){
          this.drivePosition = drivePosition;
          this.colPos = colPos;
          this.rowPos = rowPos;
        }
    }

    public static final class VortexMotorConstants {
        public static final double kFreeSpeedRpm = 6784;
      }

    public static final class PneumaticConstants {
        public static final int compressorInput = 1;
        
        public static final int launcherUp = 0;
        public static final int launcherDown = 1;

        public static final int AmpArmOut = 3;
        public static final int AmptArmIn = 2;
    }

    public static final class ClimberConstants {
        public static final int leftClimber = 36;
        public static final int rightClimber = 35;

        public static final double kSpoolDiameterMeters = Units.inchesToMeters(2); // 4
        public static final double kClimberMotorGearRatio = 1 / 64;
        public static final double kClimberEncoderRot2Meter = kClimberMotorGearRatio * Math.PI * kSpoolDiameterMeters;
    }

    public static final class TrapConstants {
        public static final double AngleOffset = 15; // 15 degree angle to right (counterclock wise)
    }

    public static final class CameraConstants {
        public static final String CameraName = "limelight"; 
        public static final int AprilTagPipeline = 0;

        public static final double RobotCameraHeight = Units.inchesToMeters(14.75);
        public static final double RobotCameraAngle = Units.degreesToRadians(30.6);
        public static final double RobotDistance = 58;
    }

    public static final class AprilTags {
        public static final ArrayList<AprilTag> AprilTags = AprilTag.LoadAprilTagList();
    }
}