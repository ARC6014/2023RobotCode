// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TelescobicSubsystem;
import frc.team6014.SuperStructureState;

public class AutoMoveArm extends CommandBase {
  private final CarriageSubsystem m_carriage = CarriageSubsystem.getInstance();
  private final ElevatorSubsystem m_elevator = ElevatorSubsystem.getInstance();
  private final TelescobicSubsystem m_telescobic = TelescobicSubsystem.getInstance();

  private final SuperStructureState m_pivotState = new SuperStructureState(124.5,92.5,0);
  private SuperStructureState targetState;

  private boolean readyToMove;
  /** Creates a new SuperStructureCalibration. */
  public AutoMoveArm(SuperStructureState state) {
    targetState = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_carriage, m_elevator, m_telescobic);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    readyToMove = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setElevatorPosition(m_pivotState);
    m_telescobic.setTelescopicPosition(m_pivotState);
    m_carriage.holdCarriagePosition();

    readyToMove = m_elevator.isAtSetpoint() && m_telescobic.isAtSetpoint();

    if(readyToMove){
      m_carriage.setCarriagePosition(targetState);
    }

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
    return m_carriage.isAtSetpoint();
  }
}
