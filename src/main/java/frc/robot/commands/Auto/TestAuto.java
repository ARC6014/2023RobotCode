// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;



import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.team6014.SwerveAutoBuilder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  private final DriveSubsystem m_drive = DriveSubsystem.getInstance();
  /** Creates a new TestAuto. */
  public TestAuto() {
    final SwerveAutoBuilder swerveCommand = new SwerveAutoBuilder("Deneme", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, false);
    final SwerveAutoBuilder swerveCommand1 = new SwerveAutoBuilder("Deneme1", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, false);
    final SwerveAutoBuilder swerveCommand2 = new SwerveAutoBuilder("Deneme2", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, false);
    final SwerveAutoBuilder swerveCommand3 = new SwerveAutoBuilder("Deneme3", AutoConstants.kMaxSpeed, AutoConstants.kMaxAcceleration, true);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand()final SwerveAutoBuilder swerveCommand = new SwerveAutoBuilder("Deneme", AutoConstants.kMaxSpeed, AutosConstants.kMaxAcceleration, true););
    addCommands(
      new InstantCommand(() -> m_drive.resetOdometry(swerveCommand.getInitialPose())),
      swerveCommand,
      swerveCommand1,
      swerveCommand2,
      swerveCommand3
    );
  }
}
