package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.utility.AprilTag;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;

public class Camera extends SubsystemBase {
    private PhotonCamera camera;

    private static Camera instance;

    // The field from AprilTagFields will be different depending on the game. only returns pose objects no height
    //private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
    private static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    //public AprilTag _aprilTag;

    private PhotonPipelineResult result;
    private double _yawVal = 0, _yawEstPoseVal = 0;
    private double _xVal = 0, _yVal = 0, _zVal = 0;
    private double _xEstPoseVal = 0, _yEstPoseVal = 0, _zEstPoseVal = 0;
    private double _poseAmbiguity = 0;
    private int _targetId = -1, _aprilTagID = -1;
    private boolean _hasTarget = false;
    private boolean _lockOnTarget;
    private Transform3d bestCameraToTarget;

    public PhotonPoseEstimator photonPoseEstimator;
    public Transform3d robotToCam;
    

    public Camera() { 
        camera = new PhotonCamera("Arducam_OV9281_USB_Camera");

        //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
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

    public static Camera getInstance() {
        if (instance == null) {
            instance = new Camera();
        }
        return instance;
    }

	//@Override()
	//public void periodic(){
    public void visionPoseEstimator(boolean lockOnTarget){

        this._lockOnTarget = lockOnTarget;


        PhotonTrackedTarget _bestTarget;

        Optional<EstimatedRobotPose> testVisionEst = Optional.empty();
		
		// Get latest frame from Camera 
		var latestResults = camera.getAllUnreadResults();
		if(!latestResults.isEmpty()){
            var result = latestResults.get(latestResults.size() - 1);

            if (result.hasTargets()) {
                if (!this._lockOnTarget){
                    _aprilTagID = -1;
				    _bestTarget = null;
                    //Sort Targest to find closest target
				    double bestX = 9999;
				    for (PhotonTrackedTarget target : result.getTargets()) {
					    if(target.getBestCameraToTarget().getX() < bestX) {
						    _bestTarget = target;
						    bestX = target.getBestCameraToTarget().getX();
					    }
				    }

				    // At least one Apriltag was seen by the camera
				    _aprilTagID = _bestTarget.getFiducialId();
				    testVisionEst = photonPoseEstimator.update(result);
    				bestCameraToTarget = _bestTarget.getBestCameraToTarget();
                    this._poseAmbiguity = _bestTarget.getPoseAmbiguity();
                    _hasTarget = true;
                } else {
                    if (_aprilTagID > -1){
                        _bestTarget = null;
                        for (PhotonTrackedTarget target : result.getTargets()) {
                            int curTagID = target.getFiducialId();
                            if (curTagID == _aprilTagID) {
                                _bestTarget = target;
                                // At least one Apriltag was seen by the camera
				                //_aprilTagID = _bestTarget.getFiducialId();
				                testVisionEst = photonPoseEstimator.update(result);
    				            bestCameraToTarget = _bestTarget.getBestCameraToTarget();
                                this._poseAmbiguity = _bestTarget.getPoseAmbiguity();
                                _hasTarget = true;
                            } else {
                                _hasTarget = false;
                            }
                        }
                        
                    } else {
                        _hasTarget = false;
                    }
                }

                if (_hasTarget){
                    this._targetId = _aprilTagID;
                    this._yawVal = bestCameraToTarget.getRotation().getZ();
                    this._xVal = bestCameraToTarget.getX();
                    this._yVal = bestCameraToTarget.getY();
                    this._zVal = bestCameraToTarget.getZ();			

                    testVisionEst.ifPresent(p->{
                        this._xEstPoseVal = p.estimatedPose.getX();
                        this._yEstPoseVal = p.estimatedPose.getY();
                        this._zEstPoseVal = p.estimatedPose.getZ();
                        this._yawEstPoseVal = p.estimatedPose.getRotation().getZ();
                    });
                }
			} else {
                _hasTarget = false;
            }

       	}
    }


    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(result);
    }

    public PhotonCamera getPhotonCamera() {
        return camera;
    }

    public double getPoseAmbiguityVal() {
        return this._poseAmbiguity;
    }

    public double getYawVal() {
        return this._yawVal;
    }

    public double getXVal() {
        return this._xVal;
    }

    public double getYVal() {
        return this._yVal;
    }

    public double getZVal() {
        return this._zVal;
    }

    public boolean hasTargets() {
        return this._hasTarget;
    }

    public int getTargetId() {
        return this._targetId;
    }

    public double getEstPoseYawVal() {
        return this._yawEstPoseVal;
    }

    public double getEstPoseXVal() {
        return this._xEstPoseVal;
    }

    public double getEstPoseYVal() {
        return this._yEstPoseVal;
    }

    public double getEstPoseZVal() {
        return this._zEstPoseVal;
    }

}
