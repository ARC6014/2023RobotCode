// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TelescobicSubsystem;
import frc.team6014.SuperStructureState;

public class AutoExtend extends CommandBase {
  private final CarriageSubsystem m_carriage = CarriageSubsystem.getInstance();
  private final ElevatorSubsystem m_elevator = ElevatorSubsystem.getInstance();
  private final TelescobicSubsystem m_telescobic = TelescobicSubsystem.getInstance();

  private SuperStructureState targetState;

  private boolean readyToExtend;
  /** Creates a new SuperStructureCalibration. */
  public AutoExtend(SuperStructureState state) {
    targetState = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_carriage, m_elevator, m_telescobic);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    readyToExtend = m_carriage.isAtSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    readyToExtend = m_carriage.isAtSetpoint();

    m_elevator.updateLastDemandedHeight(targetState.getHeight());

    if(!readyToExtend){
      m_carriage.setCarriagePosition(targetState);
      m_elevator.holdElevatorPosition();
      m_telescobic.holdTelescopicPosition();
    }
    m_carriage.setCarriagePosition(targetState);
    m_elevator.setElevatorPosition(targetState);
    m_telescobic.setTelescopicPosition(targetState);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_carriage.holdCarriagePosition();
    m_elevator.holdElevatorPosition();
    m_telescobic.holdTelescopicPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_carriage.isAtSetpoint() && m_elevator.isAtSetpoint() && m_telescobic.isAtSetpoint();
  }
}
