// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utility.CameraTest;

public class PiCameraSubsystem extends SubsystemBase {
  /** Creates a new PiCameraSubsystem. */
  //private CommandSwerveDrivetrain m_driveTrain;
  private Drive m_driveTrain;
  boolean visionOn = true;
  private boolean initPoseSet = false;
  PhotonPipelineResult leftPipeline, rightPipeline;

  private Pose2d leftVisionPose;
  private Matrix<N3,N1> leftStdDev;
  private double leftTimeStamp;

  private Pose2d rightVisionPose;
  private Matrix<N3,N1> rightStdDev;
  private double rightTimeStamp;

  public Pose2d finalVisionPose;
  private Matrix<N3,N1> finalStdDev;
  private double finalTimeStamp;

  private double maxLeftTargetArea;
  private double maxRightTargetArea;

  private CameraTest leftCamera, rightCamera;
  private Pose2d lastRobotPose;

  RobotContainer m_robot;


  // These two statements do exactly the same.  This would be needed later when we assign weights to the odometer and the vision pose estimations
  // private static final Vector<N3> visionStd = VecBuilder.fill(0.0,0.0,Units.degreesToRadians(15));
  //private static final Matrix<N3,N1> visionStdDev = MatBuilder.fill(Nat.N3(),Nat.N1(),0.1,0.1,Units.degreesToRadians(90.0));

  public PiCameraSubsystem(RobotContainer robot) {
      m_robot = robot;
      m_driveTrain = robot._drive;

      // leftCamera = new CameraTest(m_robot,"Left_Camera",Constants.VisionConstants.BOT_TO_LEFT_CAM);
      // rightCamera = new CameraTest(m_robot,"Right_Camera",Constants.VisionConstants.BOT_TO_RIGHT_CAM);

      // leftCamera = new CameraTest(m_robot,"Arducam_OV9281_Black",Constants.VisionConstants.BOT_TO_LEFT_CAM);
      // rightCamera = new CameraTest(m_robot,"Arducam_OV9281_Blue",Constants.VisionConstants.BOT_TO_RIGHT_CAM);

      leftCamera = new CameraTest(m_robot, "BlackOV9281",Constants.VisionConstants.BOT_TO_LEFT_CAM);
      //rightCamera = new CameraTest("Arducam_OV9281_Blue",Constants.VisionConstants.BOT_TO_RIGHT_CAM);
  }

  @Override
  public void periodic() {
            
    // Get the latest robot pose and publish it.
    lastRobotPose = m_driveTrain.getPose(); //getState().Pose;
    // SmartDashboard.putNumber("Vision/Robot_X",lastRobotPose.getX());
    // SmartDashboard.putNumber("Vision/Robot_Y",lastRobotPose.getY());
    // SmartDashboard.putNumber("VIsion/Robot_Heading",lastRobotPose.getRotation().getDegrees());
    
    // Get the latest pipeline results from both cameras.
    leftPipeline = leftCamera.getLatestPipelineResult();
    //rightPipeline = rightCamera.getLatestPipelineResult();
    SmartDashboard.putNumber("LatePipeSize", leftPipeline.targets.size());
    
    // Do nothing is both pipelines are empty.
    //if((leftPipeline.equals(Constants.VisionConstants.noPipeResult) && (rightPipeline.equals(Constants.VisionConstants.noPipeResult)))) {
    if(leftPipeline.equals(Constants.VisionConstants.noPipeResult)) {
      return;
    }
    
    // When the robot first starts, the robot odometer (pose) is (0,0,0) while the AprilTag estimated pose is the actual
    // position of the robot in the field.
    // Therefore, if the position of the robot is the initial position and we have a valid estimated vision position, then
    // push the estimated vision position to the odometer.  This way, both odometer and vision positions are synchronized.
    // This operation is done only once.  After this initial odometer set, updates to the odometer are performed using the 
    // drivetrain.addVisionMeasurement method.
    if(!initPoseSet) {
    
      leftVisionPose = leftCamera.getLastVisionPose(lastRobotPose,false);
      //rightVisionPose = rightCamera.getLastVisionPose(lastRobotPose, false);
    
      if(!leftVisionPose.equals(Constants.VisionConstants.noPose)) {
        m_driveTrain.resetPose(leftVisionPose);
        initPoseSet = true;                
      }
      // else if(!rightVisionPose.equals(Constants.VisionConstants.noPose)) {
      //   m_driveTrain.resetPose(rightVisionPose);
      //   initPoseSet = true;
      // }
      else {
        return;
      }       
    }
    // SmartDashboard.putBoolean("Vision/initPose", initPoseSet);
    
    leftVisionPose = leftCamera.getLastVisionPose(lastRobotPose,true);
    leftStdDev = leftCamera.getStandardDeviation();
    leftTimeStamp = leftCamera.getTimeStamp();
    
    // rightVisionPose = rightCamera.getLastVisionPose(lastRobotPose,true);
    // rightStdDev = rightCamera.getStandardDeviation();
    // rightTimeStamp = rightCamera.getTimeStamp();
    
    //if((leftVisionPose.equals(Constants.VisionConstants.noPose)) && (rightVisionPose.equals(Constants.VisionConstants.noPose))) {
    if(leftVisionPose.equals(Constants.VisionConstants.noPose)) {
      return;
    }
    
    
    
    // Using the maximum area of the AprilTags in the pipeline, make decisions on how to use the AprilTags in the pipeline.
    // If there are large area AprilTags (i.e., the robot is close to an AprilTag), then filter the pipeline to eliminate 
    // AprilTags that would result in inaccurate vision poses and set the standard deviation for the Karman filter very low
    // (i.e., trust the AprilTag estimated vision pose).  If the AprilTag is smaller but still usable, send them to the 
    // Karman filter but with a higher standard deviation (i.e., trust the vision pose estimation less).  And finally, for any
    // other AprilTag detected, use them but with a very low confidence standard deviation.
    
    maxLeftTargetArea = leftCamera.calcMaxArea();
    //maxRightTargetArea = rightCamera.calcMaxArea();
    SmartDashboard.putNumber("Vision/TargetArea", maxLeftTargetArea);
    //if((maxLeftTargetArea == -1) && (maxRightTargetArea == -1)) {
    if(maxLeftTargetArea == -1) {
      return;
    }
    
    // if(!leftVisionPose.equals(Constants.VisionConstants.noPose) && !rightVisionPose.equals(Constants.VisionConstants.noPose)) {
    //   if(maxLeftTargetArea > maxRightTargetArea) {
    //     finalVisionPose = leftVisionPose;
    //     finalStdDev = leftStdDev;
    //     finalTimeStamp = leftTimeStamp;
    //   }
    //   else {
    //     finalVisionPose = rightVisionPose;
    //     finalStdDev = rightStdDev;
    //     finalTimeStamp = rightTimeStamp;
    //   }
    // }
    //else if(!leftVisionPose.equals(Constants.VisionConstants.noPose)) {
    if(!leftVisionPose.equals(Constants.VisionConstants.noPose)) {
      finalVisionPose = leftVisionPose;
      finalStdDev = leftStdDev;
      finalTimeStamp = leftTimeStamp;            
    }
    // else if(!rightVisionPose.equals(Constants.VisionConstants.noPose)) {
    //   finalVisionPose = rightVisionPose;
    //   finalStdDev = rightStdDev;
    //   finalTimeStamp = rightTimeStamp;            
    // }
    else {
      return;
    }
    
    // Send the final vision pose estimation to the dashboard
    SmartDashboard.putNumber("Vision/FinalVisionPos_X",finalVisionPose.getX());
    SmartDashboard.putNumber("Vision/FinalVisionPos_Y",finalVisionPose.getY());
    SmartDashboard.putNumber("Vision/FinalVision_Heading",finalVisionPose.getRotation().getDegrees());
    /* 
    SmartDashboard.putNumber("Vision/Final_X_StdDev", finalStdDev.get(0,0));
    SmartDashboard.putNumber("Vision/Final_Y_StdDev", finalStdDev.get(1,0));
    SmartDashboard.putNumber("Vision/FinalHeading_StdDev", finalStdDev.get(2,0));   
    SmartDashboard.putNumber("Vision/VFinalision_TimeStamp", finalTimeStamp); 
    */
    
    
    // Check if AprilTag vision correction is enabled.  If enabled, then proceed to update the odometer
    if (visionOn)
    {
      m_driveTrain.addVisionMeasurement(finalVisionPose, Utils.fpgaToCurrentTime(finalTimeStamp),finalStdDev); // TO DO. Make corrections to the base if necessary.
      SmartDashboard.putBoolean("Updated Odometer", true);
    }
    else {
      SmartDashboard.putBoolean("Updated Odometer", false);
    }
    
    // Update the Field with the estimated vision pose and the targets on sight as they were used for updating the
    // odometer.
    //leftCamera.drawTargetsOnField();
    //rightCamera.drawTargetsOnField();
  }
    
  public Pose2d estimateScoringPosition(boolean side) {
    Pose2d scoringPose;
  
    // Estimate the scoring position from the left and right cameras perspective
    var leftScoringPosition = leftCamera.estimateScoringPosition(lastRobotPose,side);
    //var rightScoringPosition = rightCamera.estimateScoringPosition(lastRobotPose,side);
    
    // Calculate the size of the AprilTag as viewed by the left and right camera
    var leftMaxArea = leftCamera.calcMaxArea();
    //var rightMaxArea = rightCamera.calcMaxArea();
    
    // If both the left and right cameras can estimate a scoring position, then select
    // the one where the AprilTag was the largest.
    // TO-DO: verify that the largest target is the better target.
    // if((!leftScoringPosition.equals(Constants.VisionConstants.noPose)) && (!rightScoringPosition.equals(Constants.VisionConstants.noPose))) {
    //   if(leftMaxArea > rightMaxArea) {
    //     scoringPose = leftScoringPosition;
    //   }
    //   else {
    //     scoringPose = rightScoringPosition;
    //   }
    // }
    //else 
    if(!leftScoringPosition.equals(Constants.VisionConstants.noPose)) {
      scoringPose = leftScoringPosition;
    }
    // else if(!rightScoringPosition.equals(Constants.VisionConstants.noPose)) {
    //   scoringPose = rightScoringPosition;
    // }
    else {
      scoringPose = Constants.VisionConstants.noPose;
    }
    
    return scoringPose;
  }
    
  public Pose2d getRobotPose() {
    return lastRobotPose;
  }
    
  // public Pose2d estimateAlgaePosition() {
  //   Pose2d scoringPose;
    
  //   // Estimate the scoring position from the left and right cameras perspective
  //   var leftAlgaePosition = leftCamera.estimatedAlgaePose(lastRobotPose);
  //   var rightAlgaePosition = rightCamera.estimatedAlgaePose(lastRobotPose);
    
  //   // Calculate the size of the AprilTag as viewed by the left and right camera
  //   var leftMaxArea = leftCamera.calcMaxArea();
  //   var rightMaxArea = rightCamera.calcMaxArea();
    
  //   // If both the left and right cameras can estimate a scoring position, then select
  //   // the one where the AprilTag was the largest.
  //   // TO-DO: verify that the largest target is the better target.
  //   if((!leftAlgaePosition.equals(Constants.VisionConstants.noPose)) && (!rightAlgaePosition.equals(Constants.VisionConstants.noPose))) {
  //     if(leftMaxArea > rightMaxArea) {
  //       scoringPose = leftAlgaePosition;
  //     }
  //     else {
  //       scoringPose = rightAlgaePosition;
  //     }
  //   }
  //   else if(!leftAlgaePosition.equals(Constants.VisionConstants.noPose)) {
  //     scoringPose = leftAlgaePosition;
  //   }
  //   else if(!rightAlgaePosition.equals(Constants.VisionConstants.noPose)) {
  //     scoringPose = rightAlgaePosition;
  //   }
  //   else {
  //     scoringPose = Constants.VisionConstants.noPose;
  //   }
    
  //   return scoringPose;
  // }
    
  public Pose2d getL1ScoringPosition(boolean side) {
    Pose2d scoringPose;
    
    // Estimate the scoring position from the left and right cameras perspective
    var leftScoringPosition = leftCamera.getL1ScoringPosition(lastRobotPose,side);
    //var rightScoringPosition = rightCamera.getL1ScoringPosition(lastRobotPose,side);
    
    // Calculate the size of the AprilTag as viewed by the left and right camera
    var leftMaxArea = leftCamera.calcMaxArea();
    //var rightMaxArea = rightCamera.calcMaxArea();
    
    // If both the left and right cameras can estimate a scoring position, then select
    // the one where the AprilTag was the largest.
    // TO-DO: verify that the largest target is the better target.
    // if((!leftScoringPosition.equals(Constants.VisionConstants.noPose)) && (!rightScoringPosition.equals(Constants.VisionConstants.noPose))) {
    //   if(leftMaxArea > rightMaxArea) {
    //     scoringPose = leftScoringPosition;
    //   }
    //   else {
    //     scoringPose = rightScoringPosition;
    //   }
    // }
    //else 
    if(!leftScoringPosition.equals(Constants.VisionConstants.noPose)) {
      scoringPose = leftScoringPosition;
    }
    // else if(!rightScoringPosition.equals(Constants.VisionConstants.noPose)) {
    //   scoringPose = rightScoringPosition;
    // }
    else {
      scoringPose = Constants.VisionConstants.noPose;
    }
    
    return scoringPose;
  }
}

