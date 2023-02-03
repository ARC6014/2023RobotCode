// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;



/** Add your docs here. */
public class Limelight {

    private static NetworkTable limelighTable = NetworkTableInstance.getDefault().getTable("limelight");

    public static Pose2d getEstimatedPose(){
        double[] ldatas = limelighTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        return new Pose2d(ldatas[0], ldatas[1], Rotation2d.fromDegrees(ldatas[3]));
    }
}
