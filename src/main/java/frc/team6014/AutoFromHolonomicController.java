// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014;

import java.nio.file.FileSystemAlreadyExistsException;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.team6014.lib.math.AllianceFlipUtil;

public class AutoFromHolonomicController extends CommandBase {
  private DriveSubsystem m_drive = DriveSubsystem.getInstance();
  private PathPlannerTrajectory m_trajectory;
  private boolean m_preloaded;

  private final PIDController x_pid = new PIDController(AutoConstants.kPXController, 0, 0);
  private final PIDController y_pid = new PIDController(AutoConstants.kPYController, 0, 0);
  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0,
      0, AutoConstants.kThetaControllerConstraints);
  private final HolonomicDriveController m_controller = new HolonomicDriveController(x_pid, y_pid, m_thetaController);

  private Timer m_timer = new Timer();

  /** Creates a new AutoFromHolonomicController. */
  public AutoFromHolonomicController(PathPlannerTrajectory trajectory, boolean preloaded) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_trajectory = trajectory;
    m_preloaded = preloaded;

    m_controller.setTolerance(new Pose2d(0.5, 0.5, new Rotation2d().fromDegrees(5)));
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(m_drive);
  }

  // Called when t he command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_controller.setEnabled(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curTime = m_timer.get();
    PathPlannerState desiredState = (PathPlannerState) m_trajectory.sample(curTime);

    if (m_preloaded) {
      desiredState = PathPlannerTrajectory.transformStateForAlliance(desiredState, DriverStation.getAlliance());
      desiredState = AllianceFlipUtil.apply(desiredState);
    }

    System.out.println("State :  " +  desiredState.poseMeters);

    SwerveModuleState[] moduleStates = Constants.kinematics.toSwerveModuleStates(
        m_controller.calculate(AllianceFlipUtil.apply(m_drive.getPose()), desiredState, desiredState.holonomicRotation));

    m_drive.setClosedLoopStates(moduleStates);

    System.out.println("Robot Pose :  " +  m_drive.getPose());

    SmartDashboard.putBoolean("Controller ", m_controller.atReference());
    SmartDashboard.putBoolean("timeout ", m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    System.out.println("-------------------------------------------------------------------s");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controller.atReference() && m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }

}
