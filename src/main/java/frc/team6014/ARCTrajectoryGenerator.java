

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014;


import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.team6014.lib.Pathplanner.PathConstraints;
import frc.team6014.lib.Pathplanner.PathPlanner;
import frc.team6014.lib.Pathplanner.PathPlannerTrajectory;
import frc.team6014.lib.Pathplanner.PathPoint;
import frc.team6014.lib.math.AllianceFlipUtil;

/** Add your docs here. */
public class ARCTrajectoryGenerator {

    private static DriveSubsystem m_drive = DriveSubsystem.getInstance();


    public static PathPlannerTrajectory generateTrajectory(Pose2d targetPose2d){
        return PathPlanner.generatePath(new PathConstraints(AutoConstants.kMaxSpeedOnTeleop, AutoConstants.kMaxAccelerationOnTeleop),
        false,
        List.of(m_drive.getPathPoint(AutoConstants.FIRST_PIVOT_POSE2D), getPathPoint(m_drive.getPose(), AutoConstants.FIRST_PIVOT_POSE2D), getPathPoint(AutoConstants.FIRST_PIVOT_POSE2D, AutoConstants.SECOND_PIVOT_POSE2D)
        , getPathPoint(AutoConstants.SECOND_PIVOT_POSE2D, targetPose2d)
        ));
    }

    public static double getHeadingforPoints(Pose2d initialPose2d, Pose2d targetPose2d){
        double alpha = Math.atan(
            Math.abs(initialPose2d.getX() - targetPose2d.getX())/
            Math.abs(targetPose2d.getY() - initialPose2d.getY())
        );
        return alpha;
    }

    private static PathPoint getPathPoint(Pose2d initial, Pose2d pose){
        return DriverStation.getAlliance() == Alliance.Blue ?
        new PathPoint(pose.getTranslation(), Rotation2d.fromDegrees(getHeadingforPoints(initial, pose) - 90), pose.getRotation()) : 
        new PathPoint(AllianceFlipUtil.apply(pose).getTranslation(), Rotation2d.fromDegrees(-getHeadingforPoints(initial, AllianceFlipUtil.apply(pose)) - 90), AllianceFlipUtil.apply(pose.getRotation()));
    }



}