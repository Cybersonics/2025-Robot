package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.DriveConstants.FrameConstants;
import frc.robot.utility.AprilTag;

import static edu.wpi.first.units.Units.*;


public class Constants {
    public static class OperatorConstants {
        public static final int DriveController = 0;
        public static final int OpController = 1;
    }

    public static class AutoConstants {
        public static final double kPhysicalMaxSpeedMetersPerSecond = FrameConstants.kPhysicalMaxSpeedMetersPerSecond;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

        public static final double kMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;// / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;// / 5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.5; //3
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;
        //public static final double kPXController = 5;
       //public static final double kPYController = 5;
        //public static final double kPThetaController = 5;

        // public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
        //         new TrapezoidProfile.Constraints(
        //                 kMaxAngularSpeedRadiansPerSecond,
        //                 kMaxAngularAccelerationRadiansPerSecondSquared);
        // public static final double kGoToPointLinearP = 0;
        // public static final double kGoToPointLinearF = 0.5;
        // public static final double kGoToPointAngularP = 0;
        // public static final double kGoToPointAngularF = 0;

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

        // public static final shuffleBoardDrive frontLeft = new shuffleBoardDrive("LF Set Angle", 0, 0);
        // public static final shuffleBoardDrive backLeft = new shuffleBoardDrive("LB Set Angle", 0, 1);

        // public static final shuffleBoardDrive frontRight = new shuffleBoardDrive("RF Set Angle", 4, 0);
        // public static final shuffleBoardDrive backRight = new shuffleBoardDrive("RBack Set Angle", 4, 1);

        public static final class ModuleConstants {
            public static final double kWheelDiameterMeters = Units.inchesToMeters(4.1); // 4
            public static final double kDriveMotorGearRatio = 1 / 6.429;
            //public static final double kTurningMotorGearRatio = 1 / 1024;
            //public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
            //public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
            //public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
           // public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

            //public static final double maxSpeed = 6.5; // M/S

            //public static final double kPTurning = 0.5;

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
            public static final double WHEEL_BASE_WIDTH = 21.375;  // Comp Bot is 22;
            public static final double kTrackWidth = Units.inchesToMeters(WHEEL_BASE_WIDTH);

            // Distance between front and back wheels
            public static final double WHEEL_BASE_LENGTH = 17.3125; //Comp bot is 22;
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
                new PIDConstants(7.5, 0, 0)); // Rotation constants P=0.68 P=5.0, I=0, D=0  kP=2.8 .65
    }

    public static class ElevatorConstants {
      public final static int leftMotorCANId = 30;
      public final static int rightMotorCANId = 31;
      public final static int stringPotChannel = 0;
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
        
        public static final int algeaOut = 0;
        public static final int algeaIn = 1;
    }

    public static final class ClimberConstants {
        public static final int leftClimber = 36;
        public static final int rightClimber = 35;

        public static final double kSpoolDiameterMeters = Units.inchesToMeters(2); // 4
        public static final double kClimberMotorGearRatio = 1 / 64;
        public static final double kClimberEncoderRot2Meter = kClimberMotorGearRatio * Math.PI * kSpoolDiameterMeters;
    }

    public static final class CameraConstants {
        public static final String CameraName = "limelight"; 
        public static final int AprilTagPipeline = 0;

        public static final double RobotCameraHeight = Units.inchesToMeters(17.25);
        public static final double RobotCameraAngle = Units.degreesToRadians(0);
        public static final double RobotDistance = 58;
    }

    public static final class AprilTags {
        public static final ArrayList<AprilTag> AprilTags = AprilTag.LoadAprilTagList();
    }
}