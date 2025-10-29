// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ScoringPresets;
import frc.robot.subsystems.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToScoringPosition extends Command {
  /** Creates a new GoToScoringPosition. */

  // Declare global variables for this command.
  RobotContainer m_robot;
  Drive m_drivetrain;
  private boolean m_side;
  PathPlannerPath m_path;
  Pose2d endPos;
  Command m_pathCommand;
  private boolean done = false;
  private Timer m_timer = new Timer();
  
  private boolean m_debug = true; //false;
  
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

  public GoToScoringPosition(RobotContainer robot, boolean side) {
    // Populate module variables
    m_robot = robot;
    m_side = side; // True is right branch and false is left branch from the driver's perspective.
    m_drivetrain = m_robot._drive;
    m_robot.visionEnable = true;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Estimate the scoring position which becomes the end position of the
    // PathPlanner trajectory.

    // If the robot is in algae mode, then the robot needs the algae position.
    // If the robot is in coral mode, then the robot needs the coral scoring
    // position for the left or right branch.
    
    //if (m_robot.getMode() == Constants.ElevatorAndArmMode.CORAL) {
    //  if(m_robot.getScoringPreset() == Constants.ScoringPresets.L1) {
    //    endPos = m_robot.PiCamera.getL1ScoringPosition(m_side);
    //  }
    //  else {
        endPos = m_robot.PiCamera.estimateScoringPosition(m_side);
        // SmartDashboard.putBoolean("noPose", false);
        // SmartDashboard.putNumber("endPosX", endPos.getX());
        // SmartDashboard.putNumber("endPosY", endPos.getY());
    //  }
    //} else {
    //  endPos = m_robot.PiCamera.estimateAlgaePosition();
    //}

    // Test for a valid Pose2d return. If a valid scoring position was not able to be calculated.
    if (endPos.equals(Constants.VisionConstants.noPose)) {
      done = true; // Signal to end the command.
      SmartDashboard.putBoolean("noPose", true);
      return; // Stop executing any more statements and go to end. TO-DO: check if m_timer
              // cause a crash because not being initialized.
    } else { SmartDashboard.putBoolean("noPose", false);}

    // Extract the end heading of the robot with respect to the field.
    Rotation2d endRobotHeading = endPos.getRotation();

    // Get the current pose of the robot from the odometer
    Pose2d currentPose = m_drivetrain.getPose();

    // The rotation component in the robot pose represents the orientation of the robot during travel
    Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());

    if (m_debug) {
      SmartDashboard.putNumber("GoToTarget/Start_PosX", startPos.getX());
      SmartDashboard.putNumber("GoToTarget/Start_PosY", startPos.getY());
      SmartDashboard.putNumber("GoToTarget/Start_Heading", startPos.getRotation().getDegrees());

      SmartDashboard.putNumber("GoToTarget/End_PosX", endPos.getX());
      SmartDashboard.putNumber("GoToTarget/End_PosY", endPos.getY());
      SmartDashboard.putNumber("GoToTarget/End_Heading", endPos.getRotation().getDegrees());

    }

      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
      m_path = new PathPlannerPath(
          waypoints,
          new PathConstraints(
              4.5, 2.5,
              Units.degreesToRadians(270), Units.degreesToRadians(360)),
          null, // Ideal starting state can be null for on-the-fly paths
          new GoalEndState(0.0, endRobotHeading) // The rotation should be (tagHeading - 180 degrees)
      );

      //maxVelocityMPS 5.41

    // Prevent this path from being flipped on the red alliance, since the given
    // positions are absolute positions
    // with respect to the field. If this flag is false, PathPlanner will flip the
    // field if the alliance is red.
    // Flipping the field means that the origin (0,0,0) flips from the blue alliance
    // bottom corner to the red alliance
    // top corner.
    m_path.preventFlipping = true;

    try {
      // AutoBuilder.followPath(m_path).schedule();
      // This is equivalent to the one linear commented above.
      m_pathCommand = AutoBuilder.followPath(m_path);
    } catch (Exception ex) {
      // This command is done. No point in continuing execution if the path build
      // failed.
      done = true;
      DriverStation.reportError("Failed to build path command in GoToTarget command.", ex.getStackTrace());
    }

    // Schedule the PathPlanner command to run. Good luck !!!
        
    if (m_pathCommand != null) {
     m_pathCommand.schedule();
    }

    
    // Start a timer to stop this command is the timer is exceeded.
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     // If the command has been running longer than the maxTime set in Constants,
    // then cancel the command. This is important because if this command gets stuck
    // (PathPlanner path never ends), the driver will not be able to drive the
    // robot.
    // This is a failsafe feature.
    if (m_timer.get() > Constants.AutoScore.maxTime) {
      done = true; // Signal scheduler to cancel the command with the "interrupt" flag set to
                   // "true"
      return; // Don't execute any more statements.
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Execute the stop command and robot only if this command was interrupted.
    // If this command ended by completed the PathPlanner path, then do nothing (the
    // command
    // has already ended)
    if (interrupted) {
      // This will cancel the path command
      if (m_pathCommand.isScheduled()) {
        m_pathCommand.cancel();
      }
      // This will stop the robot from moving. The statement "new ChassisSpeeds()"
      // creates a ChassisSpeeds object with 0 velocity for X, Y, and theta. This
      // should stop
      // the robot movement in translation and rotation.
      m_drivetrain.processInput(0, 0, 0, true);
      m_drivetrain.stopModules();
      
      //m_robot.drivetrain.setControl(m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds()));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
// First check if the done flag was set to true. If so, this means that
    // the path command build failed so there is no point to check is the
    // m_pathCommand is finished (it never started).
    if (!done && m_pathCommand.isScheduled()) {
      if (m_pathCommand.isFinished()) {
        done = true;
      }
    }
    return done;
  }
}
