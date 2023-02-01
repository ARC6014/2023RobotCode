// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DriveByJoystick;
import frc.robot.commands.Auto.OnTheFlyPathGeneration;
import frc.robot.commands.Auto.TestAuto;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drivetrain = DriveSubsystem.getInstance();
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  Joystick driver = new Joystick(0);
  //Logger logger; 
  

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DriveByJoystick teleopDriveByJoystick = new DriveByJoystick(()-> driver.getRawAxis(1)*-1, ()-> driver.getRawAxis(0)*-1, ()-> driver.getRawAxis(2)*-1, () -> driver.getRawButton(7),() -> driver.getRawButton(8));
  //private final IntakeCommand intake = new IntakeCommand();
  //private final OuttakeCommand outtake = new OuttakeCommand();
  private final OnTheFlyPathGeneration m_tesFlyPathGeneration = new OnTheFlyPathGeneration(List.of(AutoConstants.testPoint));
  //private final AutoFromHolonomicController m_testPath = new AutoFromHolonomicController();
  //private final ChaseTag visionTest = new ChaseTag();
  //private final RunCommand swerveLock = new RunCommand(() -> m_drivetrain.changeLock(), m_drivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand(teleopDriveByJoystick);
    // Configure the button bindings
    configureButtonBindings();
   /*logger = Logger.getLoggerInstance();
    logger.addShutdownHook();*/
  
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //new JoystickButton(driver,1).whileTrue(intake);
    //new JoystickButton(driver, 2).whileTrue(outtake);
    //new JoystickButton(driver, 1).whileTrue(m_tesFlyPathGeneration);
    //new JoystickButton(driver, 2).whileTrue(visionTest);
   // new JoystickButton(driver, 1).toggleWhenPressed(swerveLock, true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new TestAuto();
  }
  
}