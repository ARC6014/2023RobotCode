// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;



import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.team6014.SwerveAutoBuilder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  private final DriveSubsystem m_drive = DriveSubsystem.getInstance();
 // private final PoseEstimatorSubsystem m_poseEstimator = PoseEstimatorSubsystem.getInstance();
  /** Creates a new TestAuto. */
  public TestAuto(boolean blueAllience) {
    //PATHS4BLUE
    final SwerveAutoBuilder blueSwerveCommand = new SwerveAutoBuilder("BLUETestAuto1", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, false);
    final SwerveAutoBuilder blueSwerveCommand1 = new SwerveAutoBuilder("BLUETestAuto2", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, false);
    final SwerveAutoBuilder blueSwerveCommand2 = new SwerveAutoBuilder("BLUETestAuto3", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, false);
    final SwerveAutoBuilder blueSwerveCommand3 = new SwerveAutoBuilder("BLUETestAuto4", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, true);
    //PATHS4RED
    final SwerveAutoBuilder redSwerveCommand = new SwerveAutoBuilder("REDTestAuto1", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, false);
    final SwerveAutoBuilder redSwerveCommand1 = new SwerveAutoBuilder("REDTestAuto2", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, false);
    final SwerveAutoBuilder redSwerveCommand2 = new SwerveAutoBuilder("REDTestAuto3", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, false);
    final SwerveAutoBuilder redSwerveCommand3 = new SwerveAutoBuilder("REDTestAuto4", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, true);
    // Add your commands in the addCommands() call, e.g.

    if(blueAllience){
      addCommands(
      new InstantCommand(() -> m_drive.resetOdometry(blueSwerveCommand.getInitialPose()), m_drive),
      blueSwerveCommand,
      blueSwerveCommand1,
      blueSwerveCommand2,
      blueSwerveCommand3
      );
    }else{
    addCommands(
      new InstantCommand(() -> m_drive.resetOdometry(redSwerveCommand.getInitialPose()), m_drive),
      redSwerveCommand,
      redSwerveCommand1,
      redSwerveCommand2,
      redSwerveCommand3
      );
    }
  }
}
