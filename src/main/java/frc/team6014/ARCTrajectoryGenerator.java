

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014;


import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.team6014.lib.math.AllianceFlipUtil;

/** Add your docs here. */
public class ARCTrajectoryGenerator {

    private static DriveSubsystem m_drive = DriveSubsystem.getInstance();
    public static final PathPoint testPoint = DriverStation.getAlliance()==Alliance.Blue ? new PathPoint(new Translation2d(0,FieldConstants.fieldWidth), new Rotation2d(0), new Rotation2d(0)) : new PathPoint(new Translation2d(0,0), new Rotation2d(0));


    public static PathPlannerTrajectory generateTrajectory(){
        /*pathpoints.add(0, new PathPoint(
            new Translation2d(m_drive.getPose().getX(), m_drive.getPose().getY()), m_drive.getRotation2d())
            );*/
        return PathPlanner.generatePath(new PathConstraints(AutoConstants.kMaxSpeedOnTeleop, AutoConstants.kMaxAccelerationOnTeleop),
        false,
        List.of(m_drive.getPathPoint(AutoConstants.testPose), getTesPathPoint())
        );
    }

    private static double getHeadingforPoint(Pose2d odometrPose2d, Pose2d targetPose2d){
        double alpha = Math.atan(
            Math.abs(odometrPose2d.getX()-targetPose2d.getX())/
            Math.abs(targetPose2d.getY()-odometrPose2d.getY())
        );
        return alpha;
    }

    public static PathPoint getTesPathPoint(){
        return DriverStation.getAlliance()==Alliance.Blue ? new PathPoint(new Translation2d(AutoConstants.testPose.getX(),AutoConstants.testPose.getY()), Rotation2d.fromDegrees(getHeadingforPoint(m_drive.getPose(), AutoConstants.testPose)), AutoConstants.testPose.getRotation()) : 
        new PathPoint(new Translation2d(AutoConstants.testPose.getX(), FieldConstants.fieldWidth - AutoConstants.testPose.getY()), Rotation2d.fromDegrees(90 - getHeadingforPoint(m_drive.getPose(), AutoConstants.testPose)).times(-1), AutoConstants.testPose.getRotation().times(-1));
    }



}