// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

/** Add your docs here. */
public class AllianceFlipUtil {
    /** Flips a translation to the correct side of the field based on the current alliance color. */
    public static Translation2d apply(Translation2d translation) {
      if (shouldFlip()) {
        return new Translation2d(FieldConstants.fieldLength - translation.getX(), translation.getY());
      } else {
        return translation;
      }
    }
  
    /** Flips a rotation based on the current alliance color. */
    public static Rotation2d apply(Rotation2d rotation) {
      if (shouldFlip()) {
        return new Rotation2d(-rotation.getCos(), rotation.getSin());
      } else {
        return rotation;
      }
    }
  
    /** Flips a pose to the correct side of the field based on the current alliance color. */
    public static Pose2d apply(Pose2d pose) {
      if (shouldFlip()) {
        return new Pose2d(
            FieldConstants.fieldLength - pose.getX(),
            pose.getY(),
            Rotation2d.fromDegrees(pose.getRotation().getDegrees() + 180));
      } else {
        return pose;
      }
    }
  
    private static boolean shouldFlip() {
      return DriverStation.getAlliance() == Alliance.Red;
    }
  }
