// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TelescobicArmSubsystem;

public class PercentOutputCommand extends CommandBase {
  private final CarriageSubsystem m_carriage = CarriageSubsystem.getInstance();
  private final ElevatorSubsystem m_elevator = ElevatorSubsystem.getInstance();
  private final TelescobicArmSubsystem m_telescobic = TelescobicArmSubsystem.getInstance();
  private Joystick m_joystick;
  private final double carraige_speed = 0;
  private final double elevator_speed = 0;
  private final double telescobic_speed = 0;
    /** Creates a new PercentOutputCommand. */
  public PercentOutputCommand(Joystick joystick) {
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(m_joystick.getRawButtonPressed(6)){
      m_carriage.setMotorOutput(carraige_speed);
    }
    if(m_joystick.getRawButtonPressed(11)){
      m_elevator.setMotorOutput(elevator_speed);
    }
    if(m_joystick.getRawButtonPressed(8)){
      m_telescobic.setMotorOutput(telescobic_speed);
    }
    if(m_joystick.getRawButtonPressed(5)){
      m_carriage.setMotorOutput(-carraige_speed);
    }
    if(m_joystick.getRawButtonPressed(12)){
      m_elevator.setMotorOutput(-elevator_speed);
    }
    if(m_joystick.getRawButtonPressed(7)){
      m_telescobic.setMotorOutput(-telescobic_speed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
