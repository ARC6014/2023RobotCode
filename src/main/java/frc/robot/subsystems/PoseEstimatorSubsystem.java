// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class PoseEstimatorSubsystem extends SubsystemBase{

private static PoseEstimatorSubsystem m_instance;

public static PoseEstimatorSubsystem getInstance(){
  if(m_instance == null){
      m_instance = new PoseEstimatorSubsystem();
  }
  return m_instance;
}

private final DriveSubsystem m_drive = DriveSubsystem.getInstance();
private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.5, 0.5, 3);
private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 0.5);


private final SwerveDrivePoseEstimator m_poseEstimator;

  /** Creates a new PoseEstimatorSubsystem. */
  public PoseEstimatorSubsystem() {
    m_poseEstimator = new SwerveDrivePoseEstimator(Constants.kinematics, m_drive.getRotation2d(), m_drive.getModulePositions(), new Pose2d(), stateStdDevs, visionMeasurementStdDevs);
  }



  @Override
  public void periodic() {
    m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_drive.getRotation2d(), m_drive.getModulePositions());

    LimelightHelpers.Results visionResult = LimelightHelpers.getLatestResults("limelight").targetingResults;

    var vpose = LimelightHelpers.toPose2D(visionResult.botpose_wpiblue);
    SmartDashboard.putString("Vision Pose", String.format("(%.3f, %.3f) %.2f degrees", 
    vpose.getX(), 
    vpose.getY(),
    vpose.getRotation().getDegrees()));

    double rot = visionResult.getBotPose2d().getRotation().getDegrees();
    //System.out.println(visionResult.getBotPose2d().getRotation().getDegrees());

    if(!(visionResult.botpose[0] == 0 && visionResult.botpose[1] == 0)
      && (visionResult.getBotPose2d().getX() <= -5.93 || visionResult.getBotPose2d().getX() >= 5.93)
      && ( (rot < 17.5 && rot > -17.5) || (rot > 162.5 || rot < -162.5) )
    ){
      //System.out.println("alo");
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