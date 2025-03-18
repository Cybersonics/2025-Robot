package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.AprilTag;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class Camera extends SubsystemBase {
    private PhotonCamera camera;

    private static Camera instance;
    private static AprilTag _aprilTags;
    // private final PhotonCamera camera;
    private final double CAMERA_HEIGHT_METERS = Constants.CameraConstants.RobotCameraHeight;
    // Angle between horizontal and the camera.
    private final double CAMERA_PITCH_RADIANS = Constants.CameraConstants.RobotCameraAngle;

    private double TARGET_HEIGHT_METERS; // Convert to meters when setting

    private PhotonPipelineResult result;
    private double yawVal = 0, pitchVal = 0, skewVal = 0, areaVal = 0;
    private int targetId = 0;
    private boolean hasTarget = false;

    public Camera(AprilTag aprilTag) {
        _aprilTags = aprilTag;
        camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    }

    public static Camera getInstance(AprilTag aprilTag) {
        if (instance == null) {
            instance = new Camera(aprilTag);
        }
        return instance;
    }

    public double getRange() {
        TARGET_HEIGHT_METERS = 0.0;
        double range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(getPitchVal()));
        double rangeInInches = Units.metersToInches(range);

        SmartDashboard.putNumber("Camera Distance", rangeInInches);

        return range;
    }

    public PhotonCamera getPhotonCamera() {
        return camera;
    }

    public double getYawVal() {
        return this.yawVal;
    }

    public double getPitchVal() {
        return this.pitchVal;
    }

    public double getSkewVal() {
        return this.skewVal;
    }

    public double getAreaVal() {
        return this.areaVal;
    }

    public boolean hasTargets() {
        return this.hasTarget;
    }

    public int getTargetId() {
        return this.targetId;
    }

    public Pose3d getRobotPostForAprilTag(int tagId) {
       return _aprilTags.getNeededRobotPost(tagId);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Camera connected", this.camera.isConnected());
    }
}
