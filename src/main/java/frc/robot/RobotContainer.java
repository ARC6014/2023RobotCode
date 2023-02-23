// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DriveByJoystick;
import frc.robot.commands.ElevatorDeneme;
import frc.robot.commands.PercentOutputCommand;
import frc.robot.commands.SuperStructureCalibration;
import frc.robot.commands.Auto.TeleopMoveToPose;
import frc.robot.commands.Auto.TestAuto;
import frc.robot.commands.Grabbing.GrabCommand;
import frc.robot.commands.Grabbing.RelaseCommand;
import frc.robot.commands.Intaking.IntakeCommand;
import frc.robot.commands.Intaking.Outtake;
import frc.robot.commands.Resetting.ZeroElevator;
import frc.robot.commands.Resetting.ZeroTelescopic;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.TelescobicArmSubsystem;
import io.github.oblarg.oblog.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drivetrain = DriveSubsystem.getInstance();
  private final PoseEstimatorSubsystem m_poseEstimatorSubsystem = PoseEstimatorSubsystem.getInstance();
  private final CarriageSubsystem m_carriage = CarriageSubsystem.getInstance();
  private final IntakeSubsytem m_intake = IntakeSubsytem.getInstance();
  private final ElevatorSubsystem m_elevator = ElevatorSubsystem.getInstance();
  private final GrabberSubsystem m_grabber = GrabberSubsystem.getInstance();
  private final RobotState m_robotState = RobotState.getInstance();
  private final TelescobicArmSubsystem m_telescopic = TelescobicArmSubsystem.getInstance();
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  Joystick driver = new Joystick(0);
  //Logger logger; 
  

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DriveByJoystick teleopDriveByJoystick = new DriveByJoystick(()-> driver.getRawAxis(1) * -1, ()-> driver.getRawAxis(0)*-1, ()-> driver.getRawAxis(2)*-1, () -> driver.getRawButton(7),() -> driver.getRawButton(8));
  private final ElevatorDeneme m_Deneme = new ElevatorDeneme(() -> driver.getRawAxis(1) * -1, ()-> driver.getRawButton(1), () -> driver.getRawButton(3));
  private final SuperStructureCalibration m_calibration = new SuperStructureCalibration();
  private final GrabCommand m_grab = new GrabCommand();
  private final RelaseCommand m_relase = new RelaseCommand();
  //private final OnTheFlyPathGeneration m_tesFlyPathGeneration = new OnTheFlyPathGeneration(AutoConstants.firstNode);
  //private final OnTheFlyPathGeneration m_tesFlyPathGeneration2 = new OnTheFlyPathGeneration(AutoConstants.secondNode);
  private final TeleopMoveToPose m_tesFlyPathGeneration1 = new TeleopMoveToPose(AutoConstants.FIRST_PIVOT_POSE2D, AutoConstants.SECOND_PIVOT_POSE2D, AutoConstants.firstNode);
  private final TeleopMoveToPose m_tesFlyPathGeneration2 = new TeleopMoveToPose(AutoConstants.FIRST_PIVOT_POSE2D, AutoConstants.SECOND_PIVOT_POSE2D, AutoConstants.secondNode);

  SendableChooser<String> autonomousChooser = new SendableChooser<String>();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    autonomousChooser.setDefaultOption("Do Nothing", "Doing Nothing...");
    autonomousChooser.addOption("Test Auto", "TestAuto");

    Logger.configureLoggingAndConfig(this, false);
    //m_drivetrain.setDefaultCommand(teleopDriveByJoystick);
    //m_carriage.setDefaultCommand(m_Deneme);
    //m_telescopic.setDefaultCommand(m_Deneme);
    // Configure the button bindings
    configureButtonBindings();
   /*logger = Logger.getLoggerInstance();
    logger.addShutdownHook();*/
  SmartDashboard.putData(autonomousChooser);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  /*   new JoystickButton(driver, 2).whileTrue(m_grab);
    new JoystickButton(driver, 3).onTrue(new ZeroTelescopic());
    new JoystickButton(driver, 4).whileTrue(m_relase);
    new JoystickButton(driver, 5).whileTrue(new IntakeCommand());
    new JoystickButton(driver, 6).onTrue(new ZeroTelescopic());

    new JoystickButton(driver, 1).onTrue(new ZeroElevator());
*/
    

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /*switch (autonomousChooser.getSelected()) {
      case "TestAuto":
        return DriverStation.getAlliance() == Alliance.Blue ? new TestAuto(true) : new TestAuto(false);
      default:
      return null;
    }*/
    return null;
  }
  
}