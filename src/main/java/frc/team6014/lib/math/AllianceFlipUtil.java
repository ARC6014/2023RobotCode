
/*
 * Inspired by team 6328: http://github.com/Mechanical-Advantage
 * Customized ARC CTRE configurations
 */

package frc.team6014.lib.math;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import frc.team6014.lib.util.CustomPathPoint;


/**
 * Utility funcitons for flipping from the blue to red alliance. By default, all translations and
 * poses in {@link FieldConstants} are stored with the origin at the rightmost point on the blue
 * alliance wall.
 */
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
          new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    } else {
      return pose;
    }
  }

    /** Flips a pathpoint to the correct side of the field based on the current alliance color. */
    public static CustomPathPoint apply(CustomPathPoint pose) {
      if (shouldFlip()) {
        return new CustomPathPoint(
          apply(pose.getTranslation())
          , pose.getholonomicRotation().times(-1));
      } else {
        return pose;
      }
    }

  /**
   * Flips a trajectory state to the correct side of the field based on the current alliance color.
   */
  public static PathPlannerState apply(PathPlannerState state) {
    if (shouldFlip()) {
      PathPlannerState mirroredState = new PathPlannerState();

      Rotation2d transformedHeading = state.poseMeters.getRotation().times(-1);
      Rotation2d transformedHolonomicRotation = state.holonomicRotation.times(-1);

      mirroredState.timeSeconds = state.timeSeconds;
      mirroredState.velocityMetersPerSecond = state.velocityMetersPerSecond;
      mirroredState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
      mirroredState.poseMeters = new Pose2d(
        FieldConstants.fieldLength - state.poseMeters.getX(), 
        state.poseMeters.getY(),
        transformedHeading);
      mirroredState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
      mirroredState.holonomicRotation = transformedHolonomicRotation;
      mirroredState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
      mirroredState.curvatureRadPerMeter = -state.curvatureRadPerMeter;

          return null;
    } else {
      return state;
    }
  }

  /** Flips a rotation sequence state based on the current alliance color. */

  private static boolean shouldFlip() {
    return DriverStation.getAlliance() == Alliance.Red;
  }
}
