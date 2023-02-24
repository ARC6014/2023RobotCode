// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.team6014.ARCTrajectoryGenerator;
import frc.team6014.lib.Pathplanner.PathPoint;
import frc.team6014.lib.math.AllianceFlipUtil;

public class PoseEstimatorSubsystem extends SubsystemBase{

//@Log.Field2d(name = "Field", rowIndex = 0, columnIndex = 2, width = 6, height = 4)
//private final Field2d m_field = new Field2d();

private static PoseEstimatorSubsystem m_instance;

private final DriveSubsystem m_drive = DriveSubsystem.getInstance();
private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));


private final SwerveDrivePoseEstimator poseEstimator;

  /** Creates a new PoseEstimatorSubsystem. */
  public PoseEstimatorSubsystem() {
    poseEstimator = new SwerveDrivePoseEstimator(Constants.kinematics, m_drive.getRotation2d(), m_drive.getModulePositions(), new Pose2d(), stateStdDevs, visionMeasurementStdDevs);
  }

  public static PoseEstimatorSubsystem getInstance(){
    if(m_instance == null){
        m_instance = new PoseEstimatorSubsystem();
    }
    return m_instance;
}

  @Override
  public void periodic() {

  }

  public void resetOdometry(Pose2d pose){
    m_drive.m_gyro.reset();
    m_drive.m_gyro.setYaw(pose.getRotation().times(DriveConstants.invertGyro? -1 : 1).getDegrees());
    poseEstimator.resetPosition(m_drive.getRotation2d(), m_drive.getModulePositions(), pose);
  }

  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public PathPoint getPathPoint(Pose2d targetPose2d){
    return DriverStation.getAlliance() == Alliance.Blue ?
    new PathPoint(new Translation2d(getPose().getX(), getPose().getY()), Rotation2d.fromDegrees(ARCTrajectoryGenerator.getHeadingforPoints(getPose(), targetPose2d) - 90) , m_drive.getRotation2d()) : 
    new PathPoint(new Translation2d(getPose().getX(), getPose().getY()), Rotation2d.fromDegrees(-ARCTrajectoryGenerator.getHeadingforPoints(getPose(), AllianceFlipUtil.apply(targetPose2d)) - 90), m_drive.getRotation2d());
}

}