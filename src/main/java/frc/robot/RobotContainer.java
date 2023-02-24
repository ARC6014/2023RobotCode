// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Deneme.CarriageDeneme;
import frc.robot.commands.Deneme.ElevatorDeneme;
import frc.robot.commands.Deneme.TelescopicDeneme;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.TelescobicSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final IntakeSubsytem m_intake = IntakeSubsytem.getInstance();
  private final GrabberSubsystem m_grabber = GrabberSubsystem.getInstance();
  private final ElevatorSubsystem m_elevator = ElevatorSubsystem.getInstance();
  private final TelescobicSubsystem m_telescop = TelescobicSubsystem.getInstance();
  private final CarriageSubsystem m_carriage = CarriageSubsystem.getInstance();
  private final Joystick m_driver = new Joystick(0);
  // The robot's subsystems and commands are defined here...

  private final ElevatorDeneme m_Eeneme = new ElevatorDeneme(() -> m_driver.getRawAxis(1) * -1, () -> m_driver.getRawButton(1), () -> m_driver.getRawButton(2));
  private final TelescopicDeneme m_Teneme = new TelescopicDeneme(() -> m_driver.getRawAxis(1) * -1, () -> m_driver.getRawButton(1), () -> m_driver.getRawButton(2));
  private final CarriageDeneme m_Aeneme = new CarriageDeneme(() -> m_driver.getRawAxis(1) * -1, () -> m_driver.getRawButton(1), () -> m_driver.getRawButton(2));


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //m_elevator.setDefaultCommand(m_Eeneme);
    //m_telescop.setDefaultCommand(m_Teneme);
    m_carriage.setDefaultCommand(m_Aeneme);
    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
