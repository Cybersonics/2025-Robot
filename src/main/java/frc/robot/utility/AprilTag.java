package frc.robot.utility;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AprilTag {

    private Alliance _alliance;

    public int REEF_AB_TAGID;
    public int REEF_CD_TAGID;
    public int REEF_EF_TAGID;
    public int REEF_GH_TAGID;
    public int REEF_IJ_TAGID;
    public int REEF_KL_TAGID;

    public Pose3d REEF_AB_ROBOTPOSE;
    public Pose3d REEF_CD_ROBOTPOSE;
    public Pose3d REEF_EF_ROBOTPOSE;
    public Pose3d REEF_GH_ROBOTPOSE;
    public Pose3d REEF_IJ_ROBOTPOSE;
    public Pose3d REEF_KL_ROBOTPOSE;

    public int CORAL_STATION_LEFT_TAGID;
    public int CORAL_STATION_RIGHT_TAGID;

    public int BARGE_NEAR_SIDE;
    public int BARGE_FAR_SIDE;

    public AprilTagFieldLayout _fieldLayout;

    public AprilTag(Supplier<Alliance> alliance) {
        this._fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        this._alliance = alliance.get();

        REEF_AB_TAGID = _alliance == Alliance.Blue ? 18 : 7;
        REEF_CD_TAGID = _alliance == Alliance.Blue ? 17 : 8;
        REEF_EF_TAGID = _alliance == Alliance.Blue ? 22 : 9;
        REEF_GH_TAGID = _alliance == Alliance.Blue ? 21 : 10;
        REEF_IJ_TAGID = _alliance == Alliance.Blue ? 20 : 11;
        REEF_KL_TAGID = _alliance == Alliance.Blue ? 19 : 6;

        CORAL_STATION_LEFT_TAGID = _alliance == Alliance.Blue ? 13 : 1;
        CORAL_STATION_RIGHT_TAGID = _alliance == Alliance.Blue ? 12 : 2;

        BARGE_NEAR_SIDE = _alliance == Alliance.Blue ? 14 : 5;
        BARGE_FAR_SIDE = _alliance == Alliance.Blue ? 4 : 15;

        double x = 0, y = 0; // Place Holder
        REEF_AB_ROBOTPOSE = _alliance == Alliance.Blue ? new Pose3d(new Pose2d(x, y, new Rotation2d(0))) : new Pose3d(new Pose2d(x, y, new Rotation2d(0)));
        REEF_CD_ROBOTPOSE = _alliance == Alliance.Blue ? new Pose3d(new Pose2d(x, y, new Rotation2d(60))) : new Pose3d(new Pose2d(x, y, new Rotation2d(-60)));
        REEF_EF_ROBOTPOSE = _alliance == Alliance.Blue ? new Pose3d(new Pose2d(x, y, new Rotation2d(120))) : new Pose3d(new Pose2d(x, y, new Rotation2d(-120)));
        REEF_GH_ROBOTPOSE = _alliance == Alliance.Blue ? new Pose3d(new Pose2d(x, y, new Rotation2d(180))) : new Pose3d(new Pose2d(x, y, new Rotation2d(180)));
        REEF_IJ_ROBOTPOSE = _alliance == Alliance.Blue ? new Pose3d(new Pose2d(x, y, new Rotation2d(-120))) : new Pose3d(new Pose2d(x, y, new Rotation2d(120)));
        REEF_KL_ROBOTPOSE = _alliance == Alliance.Blue ? new Pose3d(new Pose2d(x, y, new Rotation2d(-60))) : new Pose3d(new Pose2d(x, y, new Rotation2d(60)));
    }

    public Optional<Pose3d> getTagPose(int tagId) {
        return this._fieldLayout.getTagPose(tagId);
    }

    public Pose3d getNeededRobotPost(int tagId) {
        return new Pose3d(new Pose2d(4,2, new Rotation2d(180)));
    }
}
