package frc.robot.utility;

import java.util.ArrayList;

public class AprilTag {
    private double id;
    private double headingNeeded;
    private double distanceNeeded;
    private ReefPole leftPole;
    private ReefPole rightPole;

    public AprilTag(double id, double headingNeeded, double distanceNeeded, ReefPole leftPole, ReefPole rightPole) {
        this.id = id;
        this.headingNeeded = headingNeeded;
        this.distanceNeeded = distanceNeeded;
        this.leftPole = leftPole;
        this.rightPole = rightPole;
    }

    public double getExpectedHeading() {
        return headingNeeded;
    }

    public double getDistance() {
        return distanceNeeded;
    }

    public ReefPole getLeftPole() {
        return this.leftPole;
    }

    public ReefPole getRightPole() {
        return this.rightPole;
    }

    public static ArrayList<AprilTag> LoadAprilTagList() {
        ArrayList<AprilTag> list = new ArrayList<AprilTag>();
        list.add(new AprilTag(6, 0, 0, 
                    new ReefPole("A", 100, 200, 300, 400), 
                    new ReefPole("B", 100, 200, 300, 400))
                ); 
        list.add(new AprilTag(7, 0, 0, 
                new ReefPole("C", 100, 200, 300, 400), 
                new ReefPole("D", 100, 200, 300, 400))
        );
        list.add(new AprilTag(8, 0, 0, 
            new ReefPole("E", 100, 200, 300, 400), 
            new ReefPole("F", 100, 200, 300, 400))
        );
        list.add(new AprilTag(9, 0, 0, 
            new ReefPole("G", 100, 200, 300, 400), 
            new ReefPole("H", 100, 200, 300, 400))
        );
        list.add(new AprilTag(10, 0, 0, 
            new ReefPole("I", 100, 200, 300, 400), 
            new ReefPole("J", 100, 200, 300, 400))
        );
        list.add(new AprilTag(11, 0, 0, 
            new ReefPole("K", 100, 200, 300, 400), 
            new ReefPole("L", 100, 200, 300, 400))
        );
        return list;
    }
}
