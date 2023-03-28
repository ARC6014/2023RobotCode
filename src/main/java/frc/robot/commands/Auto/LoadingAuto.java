// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;



import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotState.intakeLevel;
import frc.robot.RobotState.scoreLevel;
import frc.robot.commands.Grabbing.GrabCommand;
import frc.robot.commands.Grabbing.RelaseCommand;
import frc.robot.commands.Intaking.IntakeCommand;
import frc.robot.commands.Resetting.ZeroTelescopic;
import frc.robot.commands.Superstructure.SmartMotion;
import frc.robot.subsystems.DriveSubsystem;
import frc.team6014.SwerveAutoBuilder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LoadingAuto extends SequentialCommandGroup {
  private final DriveSubsystem m_drive = DriveSubsystem.getInstance();
  private final boolean m_blueAllience;
 // private final PoseEstimatorSubsystem m_poseEstimator = PoseEstimatorSubsystem.getInstance();
  /** Creates a new TestAuto. */
  public LoadingAuto(boolean blueAllience) {
    m_blueAllience = blueAllience;
    //PATHS4BLUE
    final SwerveAutoBuilder blueSwerveCommand = new SwerveAutoBuilder("BlueLoadingAuto1", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, false);
    final SwerveAutoBuilder blueSwerveCommand1 = new SwerveAutoBuilder("BlueLoadingAuto2", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, false);
    final SwerveAutoBuilder blueSwerveCommand2 = new SwerveAutoBuilder("BlueLoadingAuto3", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, false);
    final SwerveAutoBuilder blueSwerveCommand3 = new SwerveAutoBuilder("BlueLoadingAuto4", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, true);

    final SmartMotion motionCommand1 = new SmartMotion();
    final SmartMotion motionCommand2 = new SmartMotion();
    final SmartMotion motionCommand3 = new SmartMotion();
    final SmartMotion motionCommand4 = new SmartMotion();
    final IntakeCommand m_IntakeCommand = new IntakeCommand();
    final IntakeCommand m_IntakeCommand1 = new IntakeCommand();
    final GrabCommand m_GrabCommand = new GrabCommand();
    final GrabCommand m_GrabCommand1 = new GrabCommand();
    final RelaseCommand m_RelaseCommand = new RelaseCommand();
    final RelaseCommand m_RelaseCommand1 = new RelaseCommand();
    final ZeroTelescopic zeroing = new ZeroTelescopic();

    //PATHS4RED
    final SwerveAutoBuilder redSwerveCommand = new SwerveAutoBuilder("RedLoadingAuto1", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, false);
    final SwerveAutoBuilder redSwerveCommand1 = new SwerveAutoBuilder("RedLoadingAuto2", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, false);
    final SwerveAutoBuilder redSwerveCommand2 = new SwerveAutoBuilder("RedLoadingAuto3", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, false);
    final SwerveAutoBuilder redSwerveCommand3 = new SwerveAutoBuilder("RedLoadingAuto4", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, true);
    // Add your commands in the addCommands() call, e.g.

    if(blueAllience){
      addCommands(
        new RunCommand(() -> m_drive.resetOdometry(blueSwerveCommand.getInitialPose()), m_drive).withTimeout(0.01),
        zeroing,
        new InstantCommand(() -> RobotState.getInstance().setCube() , RobotState.getInstance()),
        new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.SecondLevel) , RobotState.getInstance()),
        motionCommand2.withTimeout(4),
        m_RelaseCommand.withTimeout(0.8),
        new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.HOMING) , RobotState.getInstance()) ,
        motionCommand3.withTimeout(3.5),
        new RunCommand(() -> m_drive.swerveDrive(2.1, 0.0, 0, true) , m_drive).withTimeout(1.1),
        new RunCommand(() -> m_drive.swerveDrive(0.64, 0, 0, true) , m_drive).withTimeout(5),
        new RunCommand(() -> m_drive.lockSwerve(true) , m_drive).withTimeout(0.02),
        new RunCommand(() -> m_drive.swerveDrive(0, 0, 0, true) , m_drive).withTimeout(2)
        /*new InstantCommand(() -> RobotState.getInstance().setIntakeLevel(intakeLevel.ground) , RobotState.getInstance()),
        new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Intake) , RobotState.getInstance()),
        blueSwerveCommand.raceWith(motionCommand2).raceWith(m_GrabCommand).raceWith(m_IntakeCommand),
        new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.SecondLevel) , RobotState.getInstance()),
        blueSwerveCommand1.raceWith(motionCommand3),
        new WaitCommand(0.3),
        m_RelaseCommand1.withTimeout(0.8),
        new InstantCommand(() -> RobotState.getInstance().setIntakeLevel(intakeLevel.ground) , RobotState.getInstance()),
        new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Intake) , RobotState.getInstance()),
        new ParallelCommandGroup(motionCommand4, blueSwerveCommand2, m_IntakeCommand1, m_GrabCommand1),
        blueSwerveCommand3*/
        /*new ParallelCommandGroup(/*blueSwerveCommand, motionCommand1)/*,
        
        blueSwerveCommand1 ,
        blueSwerveCommand2,
        blueSwerveCommand3*/
      );
    }else{
        addCommands(
          new RunCommand(() -> m_drive.resetOdometry(redSwerveCommand.getInitialPose()), m_drive).withTimeout(0.01),
          zeroing,
          new InstantCommand(() -> RobotState.getInstance().setCube() , RobotState.getInstance()),
          new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.SecondLevel) , RobotState.getInstance()),
          motionCommand2.withTimeout(3.8),
          m_RelaseCommand.withTimeout(0.8),
          new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.HOMING) , RobotState.getInstance()),
          motionCommand3.withTimeout(3.5),
          new RunCommand(() -> m_drive.swerveDrive(2.1, 0.0, 0, true) , m_drive).withTimeout(1.1),
          new RunCommand(() -> m_drive.swerveDrive(0.64, 0, 0, true) , m_drive).withTimeout(5),
          new RunCommand(() -> m_drive.lockSwerve(true) , m_drive).withTimeout(0.01),
          new RunCommand(() -> m_drive.swerveDrive(0, 0, 0, true) , m_drive).withTimeout(2)
          /*new InstantCommand(() -> RobotState.getInstance().setIntakeLevel(intakeLevel.ground) , RobotState.getInstance()),
          new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Intake) , RobotState.getInstance()),
          blueSwerveCommand.raceWith(motionCommand2).raceWith(m_GrabCommand).raceWith(m_IntakeCommand),
          new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.SecondLevel) , RobotState.getInstance()),
          blueSwerveCommand1.raceWith(motionCommand3),
          new WaitCommand(0.3)
          m_RelaseCommand1.withTimeout(0.8),
          new InstantCommand(() -> RobotState.getInstance().setIntakeLevel(intakeLevel.ground) , RobotState.getInstance()),
          new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Intake) , RobotState.getInstance()),
          new ParallelCommandGroup(motionCommand4, blueSwerveCommand2, m_IntakeCommand1, m_GrabCommand1),
          blueSwerveCommand3*/
         /*  new ParallelCommandGroup(redSwerveCommand, motionCommand1),
          blueSwerveCommand1 ,
          blueSwerveCommand2,
          blueSwerveCommand3*/
            );
    }
  }
}
