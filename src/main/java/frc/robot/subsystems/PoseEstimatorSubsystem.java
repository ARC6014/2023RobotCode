// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.naming.spi.DirStateFactory.Result;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public static PoseEstimatorSubsystem getInstance(){
  if(m_instance == null){
      m_instance = new PoseEstimatorSubsystem();
  }
  return m_instance;
}

private final DriveSubsystem m_drive = DriveSubsystem.getInstance();
private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 2.5);
private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, 0.05);


private final SwerveDrivePoseEstimator m_poseEstimator;

  /** Creates a new PoseEstimatorSubsystem. */
  public PoseEstimatorSubsystem() {
    m_poseEstimator = new SwerveDrivePoseEstimator(Constants.kinematics, m_drive.getRotation2d(), m_drive.getModulePositions(), new Pose2d(), stateStdDevs, visionMeasurementStdDevs);
  }



  @Override
  public void periodic() {
    m_poseEstimator.update(m_drive.getRotation2d(), m_drive.getModulePositions());

    LimelightHelpers.Results visionResult = LimelightHelpers.getLatestResults("limelight").targetingResults;

    var vpose = LimelightHelpers.toPose2D(visionResult.botpose_wpiblue);
    SmartDashboard.putString("Vision Pose", String.format("(%.3f, %.3f) %.2f degrees", 
    vpose.getX(), 
    vpose.getY(),
    vpose.getRotation().getDegrees()));

    if(!(visionResult.botpose[0] == 0 && visionResult.botpose[1] == 0)){
      System.out.println("alo amk");
      var pose = LimelightHelpers.toPose2D(visionResult.botpose_wpiblue);
      m_poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - (visionResult.latency_capture / 1000) - (visionResult.latency_pipeline / 1000));
    }

    SmartDashboard.putBoolean("Vision", visionResult.valid);

    //System.out.println(getFomattedPose());
    SmartDashboard.putString("Pose Estimator", getFomattedPose());

  }

  public void resetOdometry(Pose2d pose){
    m_drive.m_gyro.reset();
    m_drive.m_gyro.setYaw(pose.getRotation().times(DriveConstants.invertGyro? -1 : 1).getDegrees());
    m_poseEstimator.resetPosition(m_drive.getRotation2d(), m_drive.getModulePositions(), pose);
  }

  private String getFomattedPose() {
    var pose = getPose();
    return String.format("(%.3f, %.3f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getPose(){
    return m_poseEstimator.getEstimatedPosition();
  }

}