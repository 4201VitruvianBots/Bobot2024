// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import static frc.robot.utils.ChargedUpNodeMask.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;
import frc.robot.utils.ModuleMap;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class FieldSim extends SubsystemBase implements AutoCloseable {
  private final SwerveDrive m_swerveDrive;

  private final Field2d m_field2d = new Field2d();
  private DriverStation.Alliance m_displayedAlliance = DriverStation.Alliance.Blue;

  private ArrayList<Pose2d> m_displayedNodes = new ArrayList<>();
  private Pose2d m_highlightedNode = new Pose2d(0, 0, new Rotation2d(0));

  private Pose2d robotPose = new Pose2d(0, 0, new Rotation2d(0));
  private Pose2d intakePose;

  public FieldSim(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;

    initSim();
  }

  public void initSim() {
    initializeScoringNodes();
  }

  public Field2d getField2d() {
    return m_field2d;
  }

  /**
   * Initialize arrays with all the scoring positions on the field based on alliance color, game
   * piece type, and if it is a coopertition node. Ideally, this is a pre-processing step that we
   * only need to do once to improve robot code performance by avoiding unnecessary repeated calls.
   */
  private void initializeScoringNodes() {
  }

  public void resetRobotPose(Pose2d pose) {
    m_field2d
        .getObject("Swerve Modules")
        .setPoses(ModuleMap.orderedValues(m_swerveDrive.getModulePoses(), new Pose2d[0]));
    m_field2d.setRobotPose(pose);
  }

  private void updateRobotPoses() {
    robotPose = m_swerveDrive.getPoseMeters();
    m_field2d.setRobotPose(robotPose);

    if (RobotBase.isSimulation()) {
      //      m_field2d
      //          .getObject("localizerTagPoses")
      //          .setPoses(m_vision.getTagPoses2d(CAMERA_SERVER.LIMELIGHT));
      //      m_field2d
      //          .getObject("localizerPoses")
      //          .setPoses(m_vision.getRobotPoses2d(CAMERA_SERVER.LIMELIGHT));
      //      m_field2d
      //          .getObject("localizerPose")
      //          .setPose(m_vision.getRobotPose2d(CAMERA_SERVER.LIMELIGHT));
    }
    //
    //    intakePose =
    //        m_swerveDrive
    //            .getPoseMeters()
    //            .transformBy(new Transform2d(null, m_swerveDrive.getHeadingRotation2d()));
    //    m_field2d.getObject("Intake Pose").setPose(intakePose);

    if (RobotBase.isSimulation()) {
      m_field2d
          .getObject("Swerve Modules")
          .setPoses(ModuleMap.orderedValues(m_swerveDrive.getModulePoses(), new Pose2d[0]));
    }
  }

  @Override
  public void periodic() {
    updateRobotPoses();

    try {
      SmartDashboard.putData("Field2d", m_field2d);
    } catch (NullPointerException e) {
      //      e.printStackTrace();
    }
  }

  @Override
  public void simulationPeriodic() {}

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {}
}
