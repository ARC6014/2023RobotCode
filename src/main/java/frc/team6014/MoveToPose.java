// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.team6014.lib.math.AllianceFlipUtil;

public class MoveToPose extends CommandBase {
  private final Timer m_timer = new Timer();
  private final DriveSubsystem m_drive = DriveSubsystem.getInstance();
  //private final PoseEstimatorSubsystem m_poseEstimatorSubsystem = PoseEstimatorSubsystem.getInstance();

  private final PIDController x_pid = new PIDController(AutoConstants.kPdriveOnTeleop, 0.0, AutoConstants.kDdriveOnTeleop);
  private final PIDController y_pid = new PIDController(AutoConstants.kPdriveOnTeleop, 0.0, AutoConstants.kDdriveOnTeleop);
  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(AutoConstants.kPturnOnTeleop, 0,
    AutoConstants.kDturnOnTeleop, AutoConstants.kThetaControllerConstraints);
  private Pose2d m_targetPose;
  private boolean m_endstationary;
  private boolean m_isFinished = false;
  
  public MoveToPose(Pose2d targetPose, boolean endstationary) {
    m_targetPose = targetPose;
    m_endstationary = endstationary;    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x_pid.reset();
    y_pid.reset();
    m_thetaController.reset(m_drive.getRotation2d().getRadians());
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_targetPose = AllianceFlipUtil.apply(m_targetPose);
    if(!m_endstationary && pivotToSkip()){
      m_isFinished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(atReference(m_targetPose)){
      if(m_endstationary){
        m_drive.swerveDrive(0, 0, 0, true);
      }else{
        m_drive.stop();
      }
      m_timer.start();
    }else{
      m_timer.reset();
      m_drive.swerveDrive(
       x_pid.calculate(m_drive.getPose().getX(), m_targetPose.getX()),
       y_pid.calculate(m_drive.getPose().getY(), m_targetPose.getY()),
       m_thetaController.calculate(m_drive.getPose().getRotation().getRadians(), m_targetPose.getRotation().getRadians()),
       true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (atReference(m_targetPose) && m_timer.get() >= AutoConstants.onTheFlyMoveTreshold) || m_isFinished;
  }

  public boolean atReference(Pose2d referencePose){
    if(Math.abs(m_drive.getPose().getX() - referencePose.getX()) <= AutoConstants.kPositionToleranceX && 
      Math.abs(m_drive.getPose().getY() - referencePose.getY()) <= AutoConstants.kPositionToleranceY &&
      Math.abs(m_drive.getPose().getRotation().getRadians() - referencePose.getRotation().getRadians()) <= AutoConstants.kRotationToleranceRadians){
      return true;
    }else {
      return false;
    }
  }

  private boolean pivotToSkip(){
    if(DriverStation.getAlliance() == Alliance.Red){
      if(m_drive.getPose().getX() - m_targetPose.getX() >= 0){
        return true;
      }
    }else{
      if(m_drive.getPose().getX() - m_targetPose.getX() <= 0){
        return true;
      }
    }
    return false;
  }
}
