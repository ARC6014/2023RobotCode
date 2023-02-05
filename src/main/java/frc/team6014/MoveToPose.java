// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014;

import java.sql.Time;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.team6014.lib.math.AllianceFlipUtil;

public class MoveToPose extends CommandBase {
  private final Timer m_timer = new Timer();
  private final DriveSubsystem m_drive = DriveSubsystem.getInstance();
  private final PoseEstimatorSubsystem m_poseEstimatorSubsystem = PoseEstimatorSubsystem.getInstance();

  private final PIDController x_pid = new PIDController(AutoConstants.kPdriveOnTeleop, 0.0, AutoConstants.kDdriveOnTeleop);
  private final PIDController y_pid = new PIDController(AutoConstants.kPdriveOnTeleop, 0.0, AutoConstants.kDdriveOnTeleop);
  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(AutoConstants.kPturnOnTeleop, 0,
    AutoConstants.kDturnOnTeleop, AutoConstants.kThetaControllerConstraints);
  private Pose2d m_targetPose;
  
  public MoveToPose(Pose2d targetPose) {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(atReference(m_drive.getPose(), m_targetPose)){
      m_drive.swerveDrive(0, 0, 0, true);
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
    return atReference(m_drive.getPose(), m_targetPose) && m_timer.get() >= AutoConstants.onTheFlyMoveTreshold;
  }

  public boolean atReference(Pose2d currentPose, Pose2d referencePose){
    if(Math.abs(currentPose.getX() - referencePose.getX()) <= AutoConstants.kPositionToleranceX && 
      Math.abs(currentPose.getY() - referencePose.getY()) <= AutoConstants.kPositionToleranceY &&
      Math.abs(currentPose.getRotation().getRadians() - referencePose.getRotation().getRadians()) <= AutoConstants.kRotationToleranceRadians){

      return true;
    }else {
      return false;
    }
  }
}
