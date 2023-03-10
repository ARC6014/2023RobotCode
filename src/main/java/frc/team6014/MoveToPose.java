// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014;


import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class MoveToPose extends CommandBase {
  private final DriveSubsystem m_drive = DriveSubsystem.getInstance();
  private final PoseEstimatorSubsystem m_poseEstimatorSubsystem = PoseEstimatorSubsystem.getInstance();

  private final ProfiledPIDController x_pid = new ProfiledPIDController(AutoConstants.kPdriveOnTeleop, 0.0, AutoConstants.kDdriveOnTeleop, AutoConstants.kTranslationConstraints);
  private final ProfiledPIDController y_pid = new ProfiledPIDController(AutoConstants.kPdriveOnTeleop, 0.0, AutoConstants.kDdriveOnTeleop, AutoConstants.kTranslationConstraints);
  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(AutoConstants.kPturnOnTeleop, 0,
    AutoConstants.kDturnOnTeleop, AutoConstants.kThetaControllerConstraints);
  private Pose2d m_targetPose;
  
  public MoveToPose(Pose2d targetPose) {
    m_targetPose = targetPose;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    x_pid.setTolerance(0.03);
    y_pid.setTolerance(0.03);
    m_thetaController.setTolerance(Math.toRadians(1.5));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    x_pid.reset(m_poseEstimatorSubsystem.getPose().getX());
    y_pid.reset(m_poseEstimatorSubsystem.getPose().getY());
    m_thetaController.reset(m_poseEstimatorSubsystem.getPose().getRotation().getRadians());
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_drive.resetOdometry(m_poseEstimatorSubsystem.getPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d currentPose = m_poseEstimatorSubsystem.getPose();

    System.out.print("ABİ");

    double xSpeed = x_pid.calculate(currentPose.getX(), m_targetPose.getX());
    if (x_pid.atSetpoint()) xSpeed = 0;
    double ySpeed = y_pid.calculate(currentPose.getY(), m_targetPose.getY());
    if (y_pid.atSetpoint()) ySpeed = 0;
    double rotation = m_thetaController.calculate(m_drive.getRotation2d().getRadians(), m_targetPose.getRotation().getRadians());
    if (m_thetaController.atSetpoint()) rotation = 0;
    m_drive.setClosedLoopStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, currentPose.getRotation()));
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setLEDMode_ForceOn("limelight");
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atReference();
  }

  public boolean atReference(){
    return x_pid.atGoal() && y_pid.atGoal() && m_thetaController.atGoal(); 
  }

}
