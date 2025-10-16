// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
/** Add your docs here. */
public class CameraTest {
    PhotonPipelineResult latestPipelineResult = new PhotonPipelineResult();
    PhotonPipelineResult lastestPrunedPipeResult = new PhotonPipelineResult();
    private Pose2d latestVisionPose2d;
    private Pose3d latestVisionPose3d;
    private double lastResultTimestamp = 0;
    Optional<EstimatedRobotPose> latestVisionPose;
    List<PhotonPipelineResult> latestPipeResultsList;

    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    RobotContainer m_robot;
    Transform3d BOT_TO_CAM, CAM_TO_BOT;

    FieldObject2d fieldVisionDetections;
    FieldObject2d fieldVisionPose;
    AprilTagFieldLayout fieldLayout;


    // These two statements do exactly the same.  This would be needed later when we assign weights to the odometer and the vision pose estimations
    // private static final Vector<N3> visionStd = VecBuilder.fill(0.0,0.0,Units.degreesToRadians(15));
    // private static final Matrix<N3,N1> visionStdDev = MatBuilder.fill(Nat.N3(),Nat.N1(),0.1,0.1,Units.degreesToRadians(90.0));
    private Matrix<N3,N1> visionStdDev;



    public CameraTest (RobotContainer robot, String camName, Transform3d botToCam) {
    //public CameraTest (String camName, Transform3d botToCam) {
        m_robot = robot;
        this.BOT_TO_CAM = botToCam;
        this.CAM_TO_BOT = BOT_TO_CAM.inverse();
        camera = new PhotonCamera(camName);

        try {
            poseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // Pose strategy.  If MULTI_TAG_PNP_ON_COPROCESSOR is selected,
                                                           // make sure that this option is activated in the camera coprocessor.
                                                           // Use the Web interface to the camera coprocessor to activate (slider button)
                BOT_TO_CAM //Robot -> camera Transform3d
            );
            
            // The default strategy is to process multiple AprilTags and compute a single pose from multiple AprilTags.
            // However, when only one AprilTag is detected, this sets up the fallback strategy 
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

            // Get the field layout to get info about AprilTags positions
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);

        } catch (IOException e) {
            System.err.println("Can't load apriltags for " + camera.getName());
            e.printStackTrace();
        }

        fieldVisionDetections = m_robot.field.getObject(camera.getName()+"/visionDetections");
        fieldVisionPose = m_robot.field.getObject(camera.getName()+"/fieldVisionPose");
    }

    public void drawTargetsOnField()
    {

        if(latestVisionPose2d == null){
            return;
        }

        if (latestVisionPose2d.equals(Constants.VisionConstants.noPose))
        {
            fieldVisionDetections.setPoses(Collections.emptyList());
            fieldVisionPose.setPoses(Collections.emptyList());
            return;
        }

        ArrayList<Pose2d> targetPoses = new ArrayList<>(latestPipelineResult.targets.size());
        for ( final var target : latestPipelineResult.targets )
        {
            var cameraToTarget = target.getBestCameraToTarget();
            var transformedToRobot = latestVisionPose3d.plus(cameraToTarget).toPose2d();
            targetPoses.add(transformedToRobot);
        }

        fieldVisionDetections.setPoses(targetPoses);
        fieldVisionPose.setPose(latestVisionPose2d);
    }


    // This method returns the Pose of the scoring position that the robot needs to drive to
    // to score corals on the coral reef.
    // It returns the scoring position or (0,0,0) if a scoring position cannot be calculated.

    public Pose2d estimateScoringPosition(Pose2d robotPose, boolean side) {
        
        int tagID = estimateBestTagID(robotPose);  // Estimate the best TagID to score.  Driver should drive closer to the intended TagID
        SmartDashboard.putNumber("MadeHere", tagID);

        // Check if there was a valid ID in the pipeline.  The estimateScoringPosition() method returns -1 if no valid tagID was found.
        if(tagID < 0) {
            // Return a (0,0,0) pose.
            return Constants.VisionConstants.noPose;
        }

        // Check the ID is a valid target ID
        // On the blue side, the tag ID must be between 17 and 22
        // On the red side, the tag ID must be between 6 and 11
        // Any tag outside of this range should not be processed and a (0,0,0)
        // Pose2d() object must be returned.

        if(!(((tagID >= 17) && (tagID <= 22)) || ((tagID >= 6) && (tagID <= 11)))) {
            return Constants.VisionConstants.noPose;
        }

        // Flip the side from driver's perspective to tag perspective for those tags
        // that are facing away from the driver.
        // For the blue alliance, the side must be flipped for these tags (facing away from driver)

        if((tagID == 20) || (tagID == 21) || (tagID == 22)){
            side = !side;
        }

        // For the red alliance, the side must be flipped for these tags (facing away from driver)
        if((tagID == 9) || (tagID == 10) || (tagID == 11)){
            side = !side;
        }

        // Return the estimated Pose2d of the scoring position
        // TO-DO: should we have additional checks for errors?
        return calculateScoringPos(side, tagID);

    }


     // This method calculates the scoring positinn from the AprilTag ID and which side of the AprilTag
    // ID the scoring position is located.
    // For the side paramters, right side is true and left side is false.

    public Pose2d calculateScoringPos(boolean side, int tagID){
        
        // Get the position of the tag in the field.
        var tagPose = fieldLayout.getTagPose(tagID);
        var tagX = tagPose.get().getX();
        var tagY = tagPose.get().getY();
        var tagTheta = tagPose.get().getRotation().getAngle(); // Heading of the AprilTag in radians (default)

        // Get the distance between the center of robot and coral reef and between the center of the AprilTag
        // to the coral branches.
        double m_scorePosDistance = Constants.AutoScore.scorePosDistance;
        double m_scorePosOffset;
        double scoreX, scoreY, scoreTheta;

        // We determine if the scoring offset is to the right or to the left of the AprilTag from the
        // AprilTag perspective. This will be the opposite of the robots persective.
        // IF to the left, the offset is negative and if to the right, the offset
        // is positive.
        // If the target is on the left side of the AprilTag from the AprilTag perspective then 
        // make it negative and use the LEFT scoring position.  Otherwise, use the RIGHT scoring position.

        if(!side) {  
            m_scorePosOffset = -Constants.AutoScore.scorePosOffset_LEFT;
        }
        else {
            m_scorePosOffset = Constants.AutoScore.scorePosOffset_RIGHT;
        }
        
        // Perform the coordinates transformation assuming that the coordinate system origin (0,0) is translated by the
        // translation delta to a new origin (x',y') and rotated to change the heading reference from the AprilTag
        // perspective to robot perspective.
        scoreX = tagX + m_scorePosDistance * Math.cos(tagTheta) - m_scorePosOffset * Math.sin(tagTheta);
        scoreY = tagY + m_scorePosDistance * Math.sin(tagTheta) + m_scorePosOffset * Math.cos(tagTheta);
        //scoreTheta = tagTheta - Math.PI;//Subtract PI (180 degrees) to reverse angle from tag view to robot view
        scoreTheta = tagTheta;//Dont subtract. Robot is backing up
        // Make sure that the angle is greater than -180 degrees (or -PI degrees) because the allowed field headings are
        // between +180 and -179.9999....
        if(scoreTheta <= -Math.PI) { scoreTheta = scoreTheta + 2 * Math.PI;}

        // Return the pose of the scoring position
        return new Pose2d(new Translation2d(scoreX, scoreY), new Rotation2d(scoreTheta));
    }


    private int estimateBestTagID(Pose2d robotPose) {
        int tagID = -1; // This will return the "tag not found" value.

        // Make a copy of the latestPipelineResult because this method will prune the pipeline and
        // we don't want to prune the global pipeline just in case it need to be used whole (unpruned)
        // for some other purposes.
        var workingPipeResult = latestPipelineResult;
        // SmartDashboard.putNumber("LatePipeSize", latestPipelineResult.targets.size());
        // SmartDashboard.putNumber("PipeSize", workingPipeResult.targets.size());
        // SmartDashboard.putNumber("tagID", tagID);
        // Do nothing if the latest pipeline result has no results.
        if(workingPipeResult.equals(Constants.VisionConstants.noPipeResult)) {
            SmartDashboard.putNumber("tagID", tagID);
            return tagID;
        }

        // Remove AprilTags that do not meet the requirements to be used for selecting a target.
        // This requirements currently include size and heading with respect to the robot.  Small AprilTags and large angle difference with
        // respect to the robot result in innaccurate vision pose estimation and hence erroneous odometer settings.
        // TO-DO: have different filters for target selection and odometer.  Right now they are the same.
        for(int j = 0; j < (workingPipeResult.targets.size());){

            // Check that the size of the AprilTag is large enough for a good vision pose estimation
            if(workingPipeResult.targets.get(j).getArea() < Constants.VisionConstants.minAprilTagSize)
            {
                workingPipeResult.targets.remove(j); 
                continue;   // Do not increment J because the list elements have been shifted up
            };

            // Check that the angle to the robot of the AprilTag is small enough for a good vision pose estimation
            int tempTagID = workingPipeResult.getTargets().get(j).getFiducialId();  // Get the AprilTag ID
            double tagHeading = fieldLayout.getTagPose(tempTagID).get().getRotation().getAngle() - Math.PI; // Get the AprilTag heading with respect to the robot (180 degrees opposite)
            if(tagHeading <= -Math.PI) { tagHeading = tagHeading + 2 * Math.PI;}  // Make sure the heading is between 180 and -179.999...

            //double robotHeading = robotPose.getRotation().getRadians();  // Get the current robot heading in radians.
            double robotHeading = robotPose.getRotation().getRadians() - Math.PI;  // Get the current robot heading in radians.


            // This is a fix for tags 7 and 21 which happen to have a 0 degree heading.  The robot will be at 180 degrees, but depending on the robot
            // heading, the heading will be reported as positive or negative (For example, 179 or -179 (which is the same as 181)).
            // A negative robot heading will result in the wrong delta between the robot heading
            // and the tag heading.  This fix makes the robot heading always positive when in front of these two tags.
            if((tempTagID == 7) || (tempTagID) == 21) {
                if(robotHeading < 0.0) {  // If the robot heading is negative clockwise
                    robotHeading = robotHeading + 2 * Math.PI;  // Express the robot heading as positive counterclockwise
                }
            }

//           System.out.println("Robot Heading " + robotHeading + " Tag Heading " + tagHeading);

            // Check if the delta between the AprilTag heading and the robot heading is below the limit
            if(Math.abs(robotHeading - tagHeading) > Constants.VisionConstants.minAprilTagAngleToRobot) {
                workingPipeResult.targets.remove(j);
                continue;  // Do not increment J because the list elements have been shifted up
            }

            j++;  // If we make it here is because the tag met the requirements.  Then move to the next tag in the list.
        }

        

        // This could be useful as a filter later on: var dist = PhotonUtils.getDistanceToPose(lastRobotPose, lastRobotPose);

        // If no targets are left in the list, then do not execute anymore statements and return a tagID of -1
        if(workingPipeResult.targets.isEmpty()){
            return tagID;
        }
        
        switch (workingPipeResult.targets.size()) {
            case 1:
                tagID = workingPipeResult.getTargets().get(0).getFiducialId();
                break;

            case 2:
                if(workingPipeResult.getTargets().get(0).getArea() > workingPipeResult.getTargets().get(1).getArea()) {
                    tagID = workingPipeResult.getTargets().get(0).getFiducialId();
                }
                else {
                    tagID = workingPipeResult.getTargets().get(1).getFiducialId();
                }
        
            default:
                // This case would indicate a problem with the vision system of the robot.  No tagID update.
                tagID = -1;
                break;
        }
        SmartDashboard.putNumber("tagID", tagID);
        return tagID;
    }

    // Calculate the maximum area of the largest AprilTag in the target list.
    public double calcMaxArea() {
        double maxArea = -1;
        double tempMaxArea;

        if(latestPipelineResult.equals(Constants.VisionConstants.noPipeResult)) {
            return maxArea;
        }

        for (int j = 0; j < (latestPipelineResult.targets.size()); j++) {

            tempMaxArea = latestPipelineResult.targets.get(j).getArea();

            if (tempMaxArea > maxArea) {
                maxArea = tempMaxArea;
            }
        }

        return maxArea;
    }

    private void filterAprilTags(Pose2d robotPose) {
        // Remove AprilTags that do not meet the requirements to be used for updating the odometer.
        // This requirements currently include size and heading with respect to the robot.  Small AprilTags and large angle difference with
        // respect to the robot result in innaccurate vision pose estimation and hence erroneous odometer settings.
        for(int j = 0; j < (latestPipelineResult.targets.size());){

            // Check that the size of the AprilTag is large enough for a good vision pose estimation
            if(latestPipelineResult.targets.get(j).getArea() < Constants.VisionConstants.minAprilTagSize)
            {
//                System.out.println("Target Area" + latestPipelineResult.targets.get(j).getArea());
                latestPipelineResult.targets.remove(j); 
                continue;   // Do not increment J because the list elements have been shifted up
            };

            // Check that the angle of the AprilTag is small enough for a good vision pose estimation
            int tempTagID = latestPipelineResult.getTargets().get(j).getFiducialId();  // Get the AprilTag ID
            double tagHeading = fieldLayout.getTagPose(tempTagID).get().getRotation().getAngle() - Math.PI; // Get the AprilTag heading with respect to the robot (180 degrees opposite)
            if(tagHeading <= -Math.PI) { tagHeading = tagHeading + 2 * Math.PI;}  // Make sure the heading is between 180 and -179.999...

            //double robotHeading = robotPose.getRotation().getRadians();  // Get the current robot heading.
            double robotHeading = robotPose.getRotation().getRadians() - Math.PI;  // Get the current robot heading minus pi as cameras are on back.

            // This is a fix for tags 7 and 21 which happen to have a 0 degree heading.  The robot will be at 180 degrees, but depending on the robot
            // heading, the heading will be reported as positive or negative.  A negative robot heading will result is the wrong delta between robot
            // and tag heading.  This fix makes the robot heading always positive.
            if((tempTagID == 7) || (tempTagID) == 21) {
                if(robotHeading < 0.0) {  // If the robot heading is negative clockwise
                    robotHeading = robotHeading + 2 * Math.PI;  // Express the robot heading as positive counterclockwise
                }
            }

 //          System.out.println("Robot Heading " + robotHeading + " Tag Heading " + tagHeading);

            if(Math.abs(robotHeading - tagHeading) > Constants.VisionConstants.minAprilTagAngleToRobot) {
                latestPipelineResult.targets.remove(j);
                continue;  // Do not increment J because the list elements have been shifted up
            }

            j++; // If we make it here is because the tag met the requirements.  Then move to the next tag in the list.
        }

    }


    // TO-DO: Combined this method with the the scoring Algae method

    public Pose2d estimatedAlgaePose(Pose2d robotPose) {

        int tagID = estimateBestTagID(robotPose);  // Estimate the best TagID to score.  Driver should drive closer to the intended TagID

        // Check if there was a valid ID in the pipeline.  The estimateScoringPosition() method returns -1 if no valid tagID was found.
        if(tagID < 0) {
            // Return a (0,0,0) pose.
            return Constants.VisionConstants.noPose;
        }

        // Check the ID is a valid target ID
        // On the blue side, the tag ID must be between 17 and 22
        // On the red side, the tag ID must be between 6 and 11
        // Any tag outside of this range should not be processed and a (0,0,0)
        // Pose2d() object must be returned.

        if(!(((tagID >= 17) && (tagID <= 22)) || ((tagID >= 6) && (tagID <= 11)))) {
            return Constants.VisionConstants.noPose;
        }

        // Return the estimated Pose2d of the scoring position
        // TO-DO: should we have additional checks for errors?
        return calculateAlgaePose(tagID);
    }

    public Pose2d calculateAlgaePose(int tagID){
        
        // Get the position of the tag in the field.
        var tagPose = fieldLayout.getTagPose(tagID);
        var tagX = tagPose.get().getX();
        var tagY = tagPose.get().getY();
        var tagTheta = tagPose.get().getRotation().getAngle(); // Heading of the AprilTag in radians (default)

        // Get the distance between the center of robot and coral reef and between the center of the AprilTag
        // to the coral branches.
        double m_scorePosDistance = Constants.AutoScore.algaePosDistance;
        double m_scorePosOffset = 0.0; // Algae is in the center of the AprilTag
        double scoreX, scoreY, scoreTheta;

     
        // Perform the coordinates transformation assuming that the coordinate system origin (0,0) is translated by the
        // translation delta to a new origin (x',y') and rotated to change the heading reference from the AprilTag
        // perspective to robot perspective.
        scoreX = tagX + m_scorePosDistance * Math.cos(tagTheta) - m_scorePosOffset * Math.sin(tagTheta);
        scoreY = tagY + m_scorePosDistance * Math.sin(tagTheta) + m_scorePosOffset * Math.cos(tagTheta);
        scoreTheta = tagTheta - Math.PI;
        
        // Make sure that the angle is greater than -180 degrees (or -PI degrees) because the allowed field headings are
        // between +180 and -179.9999....
        if(scoreTheta <= -Math.PI) { scoreTheta = scoreTheta + 2 * Math.PI;}

        // Return the pose of the scoring position
        return new Pose2d(new Translation2d(scoreX, scoreY), new Rotation2d(scoreTheta));
    }

    // This method returns the Pose of the scoring position that the robot needs to drive to
    // to score corals on the coral reef.
    // It returns the scoring position or (0,0,0) if a scoring position cannot be calculated.

    public Pose2d getL1ScoringPosition(Pose2d robotPose, boolean side) {
        
        int tagID = estimateBestTagID(robotPose);  // Estimate the best TagID to score.  Driver should drive closer to the intended TagID


        // Check if there was a valid ID in the pipeline.  The estimateScoringPosition() method returns -1 if no valid tagID was found.
        if(tagID < 0) {
            // Return a (0,0,0) pose.
            return Constants.VisionConstants.noPose;
        }

        // Check the ID is a valid target ID
        // On the blue side, the tag ID must be between 17 and 22
        // On the red side, the tag ID must be between 6 and 11
        // Any tag outside of this range should not be processed and a (0,0,0)
        // Pose2d() object must be returned.

        if(!(((tagID >= 17) && (tagID <= 22)) || ((tagID >= 6) && (tagID <= 11)))) {
            return Constants.VisionConstants.noPose;
        }

        // Flip the side from driver's perspective to tag perspective for those tags
        // that are facing away from the driver.
        // For the blue alliance, the side must be flipped for these tags (facing away from driver)

        if((tagID == 20) || (tagID == 21) || (tagID == 22)){
            side = !side;
        }

        // For the red alliance, the side must be flipped for these tags (facing away from driver)
        if((tagID == 9) || (tagID == 10) || (tagID == 11)){
            side = !side;
        }

        // Return the estimated Pose2d of the scoring position
        // TO-DO: should we have additional checks for errors?
        return calcL1ScoringPos(side, tagID);

    }

    // This method calculates the scoring position from the AprilTag ID and which side of the AprilTag
    // ID the scoring position is located.  This method is a special version for L1 only.
    // For the side paramters, right side is true and left side is false.

    public Pose2d calcL1ScoringPos(boolean side, int tagID){
        
        // Get the position of the tag in the field.
        var tagPose = fieldLayout.getTagPose(tagID);
        var tagX = tagPose.get().getX();
        var tagY = tagPose.get().getY();
        var tagTheta = tagPose.get().getRotation().getAngle(); // Heading of the AprilTag in radians (default)

        // Get the distance between the center of robot and coral reef and between the center of the AprilTag
        // to the coral branches.
        double m_scorePosDistance = Constants.AutoScore.scoreL1Distance;
        double m_scorePosOffset, m_scoreAngleOffset;
        double scoreX, scoreY, scoreTheta;

        // We determine if the scoring offset is to the right or to the left of the AprilTag from the
        // AprilTag perspective.  IF to the left, the offset is negative and if to the right, the offset
        // is positive.
        // If the target is on the left side of the AprilTag from the AprilTag perspective then 
        // make it negative and use the LEFT scoring position.  Otherwise, use the RIGHT scoring position.

        if(!side) {  
            m_scorePosOffset = -Constants.AutoScore.scoreL1Offset_LEFT;
            m_scoreAngleOffset = Math.toRadians(-Constants.AutoScore.scoreL1Oangle_LEFT);
        }
        else {
            m_scorePosOffset = Constants.AutoScore.scoreL1Offset_RIGHT;
            m_scoreAngleOffset = Math.toRadians(Constants.AutoScore.scoreL1Oangle_RIGHT);
        }
        
        // Perform the coordinates transformation assuming that the coordinate system origin (0,0) is translated by the
        // translation delta to a new origin (x',y') and rotated to change the heading reference from the AprilTag
        // perspective to robot perspective.
        scoreX = tagX + m_scorePosDistance * Math.cos(tagTheta) - m_scorePosOffset * Math.sin(tagTheta);
        scoreY = tagY + m_scorePosDistance * Math.sin(tagTheta) + m_scorePosOffset * Math.cos(tagTheta);
        scoreTheta = tagTheta + m_scoreAngleOffset - Math.PI;
        
        // Make sure that the angle is greater than -180 degrees (or -PI degrees) because the allowed field headings are
        // between +180 and -179.9999....
        if(scoreTheta <= -Math.PI) { scoreTheta = scoreTheta + 2 * Math.PI;}

        // Return the pose of the scoring position
        return new Pose2d(new Translation2d(scoreX, scoreY), new Rotation2d(scoreTheta));
    }

    // This method must be call periodically to keep the pipeline up-to-date.  Failure to call this method
    // periodically will result in outdated vision pose estimations.
    public PhotonPipelineResult getLatestPipelineResult() {

        // Get the list of latests pipelines processed by the co-processor
        latestPipeResultsList = camera.getAllUnreadResults();

        // If the list is empty, return a default pipeline result.  Otherwise,
        // resturn the latest pipeline result (assuming the first result in the list is
        // the latest result).
        if(latestPipeResultsList.isEmpty()) {
            latestPipelineResult =  Constants.VisionConstants.noPipeResult;
        }
        else {
            latestPipelineResult = latestPipeResultsList.get(latestPipeResultsList.size() - 1);
            lastResultTimestamp = latestPipelineResult.getTimestampSeconds();
        }

        return latestPipelineResult;
    }

    // This method returns the latest vision estimated pose if there are enough
    // targets to estimate it.
    public Pose2d getLastVisionPose(Pose2d robotPose, boolean filter) {

        // If the pipeline result is empty, then return a (0,0,0) pose.
        if(latestPipelineResult.equals(Constants.VisionConstants.noPipeResult)) {
            return Constants.VisionConstants.noPose;
        }

        var maxArea =calcMaxArea();

        if(maxArea > Constants.VisionConstants.minAprilTagSize) {
            if(filter) {
                filterAprilTags(robotPose);  // This will filter the undesired AprilTags from the latestPipelineResult.
            }
            visionStdDev = MatBuilder.fill(Nat.N3(),Nat.N1(),0.03,0.03,Units.degreesToRadians(90.0));

        }
        else if (maxArea > Constants.VisionConstants.midRangeAprilTagSize) {
            visionStdDev = MatBuilder.fill(Nat.N3(),Nat.N1(),0.2,0.2,Units.degreesToRadians(90.0));
        }
        else {
            visionStdDev = MatBuilder.fill(Nat.N3(),Nat.N1(),0.4,0.4,Units.degreesToRadians(90.0));            
        }

        // This may not be necessary
        //        lastResultTimestamp = latestPipelineResult.getTimestampSeconds();

        // Feed the latest pipeline results to the pose estimator to get an
        // estimated vision pose.
        latestVisionPose = poseEstimator.update(latestPipelineResult);

        // If there is no pose estimation, then return a (0,0,0) pose
        if(latestVisionPose.isEmpty()) {
            //System.out.println("Estimated Pose Failed");
            return Constants.VisionConstants.noPose;
        }

        latestVisionPose3d = latestVisionPose.get().estimatedPose; 
        latestVisionPose2d = latestVisionPose3d.toPose2d();

        SmartDashboard.putNumber("Vision/"+ camera.getName() + "/Pos_X",latestVisionPose2d.getX());
        SmartDashboard.putNumber("Vision/"+ camera.getName() + "/Pos_Y",latestVisionPose2d.getY());
        SmartDashboard.putNumber("Vision/"+ camera.getName() + "/Vision_Heading",latestVisionPose2d.getRotation().getDegrees());
        SmartDashboard.putNumber("Vision/"+ camera.getName() + "/X_StdDev", visionStdDev.get(0,0));
        SmartDashboard.putNumber("Vision/"+ camera.getName() + "/Y_StdDev", visionStdDev.get(1,0));
        SmartDashboard.putNumber("Vision/"+ camera.getName() + "/Heading_StdDev", visionStdDev.get(2,0));      
        SmartDashboard.putNumber("Vision/"+ camera.getName() + "/Vision_TimeStamp", lastResultTimestamp);
        
        // If we make it here is because there is a valid vision pose estimation and
        // we can return the estimated vision pose.
        return latestVisionPose2d;
    }

    public Matrix<N3,N1> getStandardDeviation() {
        return visionStdDev;
    }

    public double getTimeStamp() {
        return lastResultTimestamp;
    }
}

