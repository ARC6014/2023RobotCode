// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.team6014.SuperStructureState;

public class SmartMove extends CommandBase {
  private final ElevatorSubsystem m_elevator = ElevatorSubsystem.getInstance();
  private final CarriageSubsystem m_carriage = CarriageSubsystem.getInstance();

  private SuperStructureState m_currentState;
  private final SuperStructureState m_targetState;
  private final SuperStructureState m_pivotState = new SuperStructureState(124,93,0);
  private boolean needToSwitch = true;
  /** Creates a new SmartMove. */
  public SmartMove(SuperStructureState targetState) {
    m_currentState = RobotState.getInstance().getCurrentSuperStructureState();
    m_targetState = targetState;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentState = RobotState.getInstance().getCurrentSuperStructureState();
    needToSwitch = m_currentState.isInSameSide(m_targetState);
    if(needToSwitch){
      m_carriage.slowCarriage();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentState = RobotState.getInstance().getCurrentSuperStructureState();
    needToSwitch = m_currentState.isInSameSide(m_targetState);
    if(needToSwitch){
      m_elevator.setElevatorPosition(m_pivotState);
      m_carriage.setCarriagePosition(m_targetState);
      if(isDeadzone()){
        m_carriage.stop();
      }
    }else{
      m_carriage.setCarriagePosition(m_targetState);
      m_elevator.setElevatorPosition(m_targetState);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.holdElevatorPosition();
    m_carriage.holdCarriagePosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_currentState.isAtDesiredState(m_targetState);
  }

  public boolean isDeadzone(){
    if(m_currentState.getAbsoluteHeight() > 30) return false;
    return true;
  }
}
