// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.management.relation.RelationException;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotState.intakeLevel;
import frc.robot.RobotState.scoreLevel;
import frc.robot.commands.DriveByJoystick;
import frc.robot.commands.SetLedState;
import frc.robot.commands.Auto.SideAuto;
import frc.robot.commands.Auto.LoadingAuto;
import frc.robot.commands.Deneme.CarriageDeneme;
import frc.robot.commands.Deneme.ElevatorDeneme;
import frc.robot.commands.Deneme.TelescopicDeneme;
import frc.robot.commands.Grabbing.GrabCommand;
import frc.robot.commands.Grabbing.RelaseCommand;
import frc.robot.commands.Intaking.IntakeCommand;
import frc.robot.commands.Intaking.Outtake;
import frc.robot.commands.Resetting.ZeroElevator;
import frc.robot.commands.Resetting.ZeroTelescopic;
import frc.robot.commands.Superstructure.AutoIntake;
import frc.robot.commands.Superstructure.AutoScore;
import frc.robot.commands.Superstructure.SmartMotion;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.TelescobicSubsystem;
import frc.robot.subsystems.UsbCam;
import frc.team6014.MoveToPose;
import frc.team6014.lib.drivers.AddressableLed;

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
  private final RobotState m_robotState = RobotState.getInstance();
  private final Joystick m_driver = new Joystick(0);
  private final Joystick m_operator = new Joystick(1);
  private final UsbCam m_cam = new UsbCam();
  private final AddressableLed m_ledRight = new AddressableLed(7);
  // The robot's subsystems and commands are defined here...

  private final DriveByJoystick driveByJoystick = new DriveByJoystick(() -> m_driver.getRawAxis(1) * -1, () -> m_driver.getRawAxis(0) * -1, () -> m_driver.getRawAxis(2) * -1, () -> m_driver.getRawButton(7), () -> m_driver.getRawButton(8), () -> m_operator.getRawButton(9));
  private final LoadingAuto blueLoadingAuto = new LoadingAuto(true);
  private final LoadingAuto redLoadingAuto = new LoadingAuto(false);
  private final SideAuto blueSideAuto = new SideAuto(true);
  private final SideAuto redSideAuto = new SideAuto(false);
  private final MoveToPose m_autoMove = new MoveToPose(()-> RobotState.getInstance().getTargetPose());
  private final MoveToPose m_SautoMove = new MoveToPose(()-> RobotState.getInstance().getScorePose());
  private final SendableChooser<String> autonomouChooser = new SendableChooser<String>();

  private final ElevatorDeneme m_Eeneme = new ElevatorDeneme(() -> m_driver.getRawAxis(1) * -1, () -> m_driver.getRawButton(1), () -> m_driver.getRawButton(2));
  private final TelescopicDeneme m_Teneme = new TelescopicDeneme(() -> m_driver.getRawAxis(1) * -1, () -> m_driver.getRawButton(1), () -> m_driver.getRawButton(2));
  private final CarriageDeneme m_Aeneme = new CarriageDeneme(() -> m_driver.getRawAxis(1) * -1, () -> m_driver.getRawButton(1), () -> m_driver.getRawButton(2));

  //private final SmartMotion m_superStructure = new SmartMotion();
  private final IntakeCommand m_intaking = new IntakeCommand();
  private final RelaseCommand m_RelaseCommand = new RelaseCommand();
  private final SmartMotion m_motion = new SmartMotion();
  private final SmartMotion m_motion1 = new SmartMotion();
  private final SmartMotion m_motion2 = new SmartMotion();
  private final AutoScore autoScore = new AutoScore();
  private final AutoIntake autoIntake = new AutoIntake();

  private final NodeSelector m_nodeSelector = NodeSelector.getInstance();
  private final RobotState m_state = RobotState.getInstance();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autonomouChooser.setDefaultOption("Do Nothing", "Nothing");
    autonomouChooser.addOption("Loading Zone", "loadingZone");
    autonomouChooser.addOption("Side ", "side");

    m_drive.setDefaultCommand(driveByJoystick);
    //m_elevator.setDefaultCommand(m_Eeneme);
    //m_telescop.setDefaultCommand(m_Teneme);
    //m_carriage.setDefaultCommand(m_Aeneme);
    m_ledRight.setDefaultCommand(new SetLedState(m_ledRight));
    // Configure the trigger bindings
    m_nodeSelector.ConfigureWidgets();

    SmartDashboard.putData(autonomouChooser);
    
    configureBindings();
  }


  private void configureBindings() {
    new JoystickButton(m_operator, 4).whileTrue(m_autoMove);
    new JoystickButton(m_operator, 6).whileTrue(m_SautoMove);

    /*new JoystickButton(m_operator, 1).whileTrue(autoIntake).toggleOnFalse(
      new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.HOMING)).andThen(
      m_motion));*/
    
    new JoystickButton(m_driver, 5).onTrue(new ZeroTelescopic());
    new JoystickButton(m_driver, 1).whileTrue(autoScore);
    new JoystickButton(m_driver, 3).whileTrue(m_motion1);
  
    new JoystickButton(m_operator, 3).whileTrue(new ParallelCommandGroup(
      new InstantCommand(() -> ElevatorSubsystem.getInstance().stop(), ElevatorSubsystem.getInstance()),
      new InstantCommand(() -> CarriageSubsystem.getInstance().stop(), CarriageSubsystem.getInstance()),
      new InstantCommand(() -> TelescobicSubsystem.getInstance().stop(), TelescobicSubsystem.getInstance())
    ));
    new JoystickButton(m_operator, 2).whileTrue(new SequentialCommandGroup(
      new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.HOMING), RobotState.getInstance()),
      m_motion
    ));
    new JoystickButton(m_driver, 6).whileTrue(new SequentialCommandGroup(
      new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.kStarting), RobotState.getInstance()),
      m_motion2
    ));
    new JoystickButton(m_operator, 1).whileTrue(new ParallelCommandGroup(m_intaking, new GrabCommand()));
    new JoystickButton(m_operator, 5).whileTrue(new ParallelCommandGroup(new Outtake(), new RelaseCommand()));

    new JoystickButton(m_driver, 4).whileTrue(m_RelaseCommand);

    new JoystickButton(m_operator, 10).whileTrue(new SequentialCommandGroup(
      new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Intake), RobotState.getInstance()),
      new InstantCommand(() -> RobotState.getInstance().setIntakeLevel(intakeLevel.ground), RobotState.getInstance()),
      new InstantCommand(() -> RobotState.getInstance().setCube(), RobotState.getInstance()),
      new SmartMotion()
    ));
    new JoystickButton(m_operator, 12).whileTrue(new SequentialCommandGroup(
      new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Intake), RobotState.getInstance()),
      new InstantCommand(() -> RobotState.getInstance().setIntakeLevel(intakeLevel.doubleStation), RobotState.getInstance()),
      new InstantCommand(() -> RobotState.getInstance().setCone(), RobotState.getInstance()),
      new SmartMotion()
    ));
 
    /*new JoystickButton(m_driver, 5).onTrue(new ZeroTelescopic());
    new JoystickButton(m_driver, 1).whileTrue(m_motion);
    new JoystickButton(m_driver, 4).whileTrue(m_intaking);
    new JoystickButton(m_driver, 2).whileTrue(m_RelaseCommand);

    //new JoystickButton(m_driver, 8).onTrue(new RunCommand(()-> m_intake.extendIntake(), m_intake));
    //new JoystickButton(m_driver, 7).onTrue(new RunCommand(()-> m_intake.retractIntake(), m_intake));
    new JoystickButton(m_driver, 5).onTrue(new ZeroTelescopic());
    new JoystickButton(m_operator, 1).whileTrue(new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.HOMING)));*/

    

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

    //new JoystickButton(m_driver, 5).onTrue(new ZeroTelescopic());

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    /*switch(autonomouChooser.getSelected()){
      case "side":
        return DriverStation.getAlliance() == Alliance.Blue? blueSideAuto : redSideAuto;
      case "loadingZone":
        return DriverStation.getAlliance() == Alliance.Blue? blueLoadingAuto : redLoadingAuto;
      default:
        return null;
    }*/
    return redLoadingAuto;

    
  }
}
