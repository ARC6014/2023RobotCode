// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;



/** Add your docs here. */
public class Limelight {

    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public static Pose2d getEstimatedPose(){

        double[] bot_pose_blue = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
  
        double tx = bot_pose_blue[0];
        double ty = bot_pose_blue[1];
        /*double tz = bot_pose_blue[2];
        double rx = bot_pose_blue[3];
        double ry = bot_pose_blue[4];*/
        double rz = (bot_pose_blue[5] + 360) % 360;
        
        return new Pose2d(tx, ty, Rotation2d.fromRadians(rz));
    }

    public static double getTimeStamp(){
        double tl = table.getEntry("tl").getDouble(0.0);
        return Timer.getFPGATimestamp() - Units.millisecondsToSeconds(tl);
    } 

    public static boolean hasValid(){
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    } 
}
