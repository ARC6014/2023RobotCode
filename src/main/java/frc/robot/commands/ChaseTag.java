// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

/*import java.util.Optional;

import com.kauailabs.navx.IMUProtocol.YPRUpdate;

//import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;*/
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
/* import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;*/
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.util.sendable.Sendable;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ChaseTag extends CommandBase {
  private DriveSubsystem m_drive = DriveSubsystem.getInstance();
  private static final TrapezoidProfile.Constraints P_CONSTRAINTS = new TrapezoidProfile.Constraints(3.5, 3);
  //private static final int TAG_TO_CHASE = 31;
  /*private static final Transform3d TAG_TO_GOAL = 
  new Transform3d(
      new Translation3d(1.5, 0, 0.0),

      new Rotation3d(0.0, 0.0, Math.PI));*/

  private final ProfiledPIDController x_pid = new ProfiledPIDController(1.5, 0, 0, P_CONSTRAINTS);
  private final ProfiledPIDController y_pid = new ProfiledPIDController(1.5, 0, 0, P_CONSTRAINTS);
  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(2, 0, 0, AutoConstants.kThetaControllerConstraints);

//  private PhotonTrackedTarget lastTarget;
  
  public ChaseTag() {
    x_pid.setTolerance(0.3);
    y_pid.setTolerance(0.3);
    m_thetaController.setTolerance(Math.toRadians(5));
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 //   lastTarget = null;
    Pose2d robotPose = m_drive.getPose();
    x_pid.reset(robotPose.getX());
    y_pid.reset(robotPose.getY());
    m_thetaController.reset(robotPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

/*     var initRobotPose2d = m_drive.getPose();
    Pose3d referenceRobotPose = new Pose3d(
      initRobotPose2d.getX(),
      initRobotPose2d.getY(),
      0,
      new Rotation3d(0,0, initRobotPose2d.getRotation().getRadians())
    );

    System.out.println(Limelight.getResult().hasTargets());

    if(Limelight.getResult().hasTargets()){



      PhotonTrackedTarget target = Limelight.getResult().getBestTarget();
      
        Transform3d camToTarget = target.getBestCameraToTarget();

        Pose3d targetPose = referenceRobotPose.transformBy(camToTarget);

        Pose2d goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

        x_pid.setGoal(goalPose.getX());
        y_pid.setGoal(goalPose.getY());
        m_thetaController.setGoal(goalPose.getRotation().getRadians());

        lastTarget = target;
      
      
    }

      double xSpeed = x_pid.calculate(initRobotPose2d.getX());
      if (x_pid.atGoal()) {
        xSpeed = 0;
      }

      double ySpeed = y_pid.calculate(initRobotPose2d.getY());
      if (y_pid.atGoal()) {
        ySpeed = 0;
      }

      double omegaSpeed = m_thetaController.calculate(initRobotPose2d.getRotation().getRadians());
      if (m_thetaController.atGoal()) {
        omegaSpeed = 0;
      }
    if(!Limelight.getResult().hasTargets()){
      m_drive.stop();
    }

      m_drive.swerveDrive(xSpeed, ySpeed, omegaSpeed, true);

      SmartDashboard.putNumber("x", xSpeed);
      SmartDashboard.putNumber("y", ySpeed);
      SmartDashboard.putNumber("rotation", omegaSpeed);

    
    
*/


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
