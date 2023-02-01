// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveAutoBuilder extends SequentialCommandGroup {
  private PathPlannerTrajectory m_trajectory;
  private DriveSubsystem m_drive = DriveSubsystem.getInstance();
  /** Creates a new SwerveAutoBuilder. */
  public SwerveAutoBuilder(String pathName, double maxSpeed, double maxAcceleration, boolean endStatinoary) {
    m_trajectory = PathPlanner.loadPath(pathName, new PathConstraints(maxSpeed, maxAcceleration));

    AutoFromHolonomicController swerveControllerCommand = new AutoFromHolonomicController(m_trajectory, true);

    if(endStatinoary){
      addCommands(
        new InstantCommand(() -> SmartDashboard.putString("AutoPath", pathName)),
         swerveControllerCommand, 
         new InstantCommand(() -> m_drive.stop()));
    }else{
      addCommands(new InstantCommand(() -> SmartDashboard.putString("AutoPath", pathName)),
          swerveControllerCommand);
    }

  }

  public Pose2d getInitialPose() {
    return m_trajectory.getInitialHolonomicPose();
  }

}
