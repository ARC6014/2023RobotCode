// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotState.scoreLevel;
import frc.robot.commands.DriveByJoystick;
import frc.robot.commands.Deneme.CarriageDeneme;
import frc.robot.commands.Deneme.ElevatorDeneme;
import frc.robot.commands.Deneme.TelescopicDeneme;
import frc.robot.commands.Grabbing.RelaseCommand;
import frc.robot.commands.Intaking.IntakeCommand;
import frc.robot.commands.Resetting.ZeroTelescopic;
import frc.robot.commands.Superstructure.SmartMotion;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.TelescobicSubsystem;
import frc.team6014.MoveToPose;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem m_drive = DriveSubsystem.getInstance();
  private final PoseEstimatorSubsystem m_poseEstimator = PoseEstimatorSubsystem.getInstance(); 
  private final IntakeSubsytem m_intake = IntakeSubsytem.getInstance();
  private final GrabberSubsystem m_grabber = GrabberSubsystem.getInstance();
  private final ElevatorSubsystem m_elevator = ElevatorSubsystem.getInstance();
  private final TelescobicSubsystem m_telescop = TelescobicSubsystem.getInstance();
  private final CarriageSubsystem m_carriage = CarriageSubsystem.getInstance();
  private final Joystick m_driver = new Joystick(0);
  private final Joystick m_operator = new Joystick(1);
  // The robot's subsystems and commands are defined here...

  private final DriveByJoystick driveByJoystick = new DriveByJoystick(() -> m_driver.getRawAxis(1) * -1, () -> m_driver.getRawAxis(0) * -1, () -> m_driver.getRawAxis(2) * -1, () -> m_driver.getRawButton(7), () -> m_driver.getRawButton(8));
  private final MoveToPose m_autoMove = new MoveToPose(()-> RobotState.getInstance().getTargetPose());
  private final MoveToPose m_SautoMove = new MoveToPose(()-> RobotState.getInstance().getScorePose());

  private final ElevatorDeneme m_Eeneme = new ElevatorDeneme(() -> m_driver.getRawAxis(1) * -1, () -> m_driver.getRawButton(1), () -> m_driver.getRawButton(2));
  private final TelescopicDeneme m_Teneme = new TelescopicDeneme(() -> m_driver.getRawAxis(1) * -1, () -> m_driver.getRawButton(1), () -> m_driver.getRawButton(2));
  private final CarriageDeneme m_Aeneme = new CarriageDeneme(() -> m_driver.getRawAxis(1) * -1, () -> m_driver.getRawButton(1), () -> m_driver.getRawButton(2));

  //private final SmartMotion m_superStructure = new SmartMotion();
  private final IntakeCommand m_intaking = new IntakeCommand();
  private final RelaseCommand m_RelaseCommand = new RelaseCommand();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //m_drive.setDefaultCommand(driveByJoystick);
    //m_elevator.setDefaultCommand(m_Eeneme);
    m_telescop.setDefaultCommand(m_Teneme);
    //m_carriage.setDefaultCommand(m_Aeneme);
    // Configure the trigger bindings

    configureBindings();
  }


  private void configureBindings() {
    /* 
    switch(robotStateSelector.getSelected()){
      case 1:
        RobotState.getInstance().setCube();
      default:
        RobotState.getInstance().setCone();
    }*/

    /*new JoystickButton(m_operator, 1).whileTrue(new InstantCommand(() -> RobotState.getInstance().setCone()));
    new JoystickButton(m_operator, 2).whileTrue(new InstantCommand(() -> RobotState.getInstance().setCube()));

    new JoystickButton(m_operator, 11).whileTrue(new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)));
    new JoystickButton(m_operator, 9).whileTrue(new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.FirstLevel)));
    new JoystickButton(m_operator, 7).whileTrue(new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.SecondLevel)));
    new JoystickButton(m_operator, 3).whileTrue(new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Intake)));
    new JoystickButton(m_operator, 4).whileTrue(new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.HOMING)));

    new JoystickButton(m_driver, 5).onTrue(new ZeroTelescopic());
    //new JoystickButton(m_driver, 6).onTrue(new ZeroElevator());
    new JoystickButton(m_operator, 8).whileTrue(m_superStructure);*/
    /*new JoystickButton(m_driver, 4).whileTrue(m_intaking);
    new JoystickButton(m_driver, 2).whileTrue(m_RelaseCommand);

    new JoystickButton(m_driver, 1).whileTrue(m_autoMove);
    new JoystickButton(m_driver, 2).whileTrue(m_SautoMove);*/

    new JoystickButton(m_driver, 5).onTrue(new ZeroTelescopic());

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;//DriverStation.getAlliance() == Alliance.Blue ? new TestAuto(true) : new TestAuto(false);
  }
}
