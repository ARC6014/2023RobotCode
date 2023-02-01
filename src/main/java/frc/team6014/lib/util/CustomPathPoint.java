package frc.team6014.lib.util;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class CustomPathPoint extends PathPoint {

  protected double prevControlLength = -1;
  protected double nextControlLength = -1;

  public CustomPathPoint(
      Translation2d position,
      Rotation2d heading,
      Rotation2d holonomicRotation,
      double velocityOverride) {
    super(position, heading, holonomicRotation, velocityOverride);
  }

  public CustomPathPoint(Translation2d position, Rotation2d heading, Rotation2d holonomicRotation) {
    this(position, heading, holonomicRotation, -1);
  }

  public CustomPathPoint(Translation2d position, Rotation2d heading, double velocityOverride) {
    this(position, heading, Rotation2d.fromDegrees(0), velocityOverride);
  }

  public CustomPathPoint(Translation2d position, Rotation2d heading) {
    this(position, heading, Rotation2d.fromDegrees(0));
  }

  public PathPoint withPrevControlLength(double lengthMeters) {
    if (lengthMeters <= 0) {
      throw new IllegalArgumentException("Control point lengths must be > 0");
    }

    prevControlLength = lengthMeters;
    return this;
  }

  public PathPoint withNextControlLength(double lengthMeters) {
    if (lengthMeters <= 0) {
      throw new IllegalArgumentException("Control point lengths must be > 0");
    }

    nextControlLength = lengthMeters;
    return this;
  }

  public PathPoint withControlLengths(
      double prevControlLengthMeters, double nextControlLengthMeters) {
    if (prevControlLengthMeters <= 0 || nextControlLengthMeters <= 0) {
      throw new IllegalArgumentException("Control point lengths must be > 0");
    }

    prevControlLength = prevControlLengthMeters;
    nextControlLength = nextControlLengthMeters;
    return this;
  }

  public Translation2d getTranslation(){
    return position;
  }

  public Rotation2d getRotation(){
    return heading;
  }

  public Rotation2d getholonomicRotation(){
    return holonomicRotation;
  }

  public static PathPoint fromCurrentHolonomicState(
      Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    double linearVel =
        Math.sqrt(
            (currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond)
                + (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));
    Rotation2d heading =
        new Rotation2d(
            Math.atan2(currentSpeeds.vyMetersPerSecond, currentSpeeds.vxMetersPerSecond));
    return new PathPoint(
        currentPose.getTranslation(), heading, currentPose.getRotation(), linearVel);
  }

  public static PathPoint fromCurrentDifferentialState(
      Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    return new PathPoint(
        currentPose.getTranslation(), currentPose.getRotation(), currentSpeeds.vxMetersPerSecond);
  }
}