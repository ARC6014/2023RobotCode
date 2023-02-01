

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014;


import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.team6014.lib.math.AllianceFlipUtil;
import frc.team6014.lib.util.CustomPathPoint;

/** Add your docs here. */
public class ARCTrajectoryGenerator {

    private static DriveSubsystem m_drive = DriveSubsystem.getInstance();


    public static PathPlannerTrajectory generateTrajectory(){
        /*for(int i = 0; i < pathpoints.size(); i++){
            pathpoints.set(i, AllianceFlipUtil.apply((CustomPathPoint) pathpoints.get(i)));
        }*/
        /*pathpoints.add(0, new PathPoint(
            new Translation2d(m_drive.getPose().getX(), m_drive.getPose().getY()), m_drive.getRotation2d())
            );*/
        return PathPlanner.generatePath(new PathConstraints(AutoConstants.kMaxSpeedOnTeleop, AutoConstants.kMaxAccelerationOnTeleop),
        false,
        List.of(new PathPoint(new Translation2d(5,5), new Rotation2d()), AutoConstants.testPoint)
        );
    }

}