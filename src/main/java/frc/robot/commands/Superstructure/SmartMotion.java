// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.RobotState.scoreLevel;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.TelescobicSubsystem;
import frc.team6014.SuperStructureState;

public class SmartMotion extends CommandBase {

  private final ElevatorSubsystem m_elevator = ElevatorSubsystem.getInstance();
  private final CarriageSubsystem m_carriage = CarriageSubsystem.getInstance();
  private final TelescobicSubsystem m_telescop = TelescobicSubsystem.getInstance();
  private final IntakeSubsytem m_intake = IntakeSubsytem.getInstance();

  private SuperStructureState m_currentState;
  private SuperStructureState m_targetState;
  private final SuperStructureState m_pivotState = new SuperStructureState(126,94.4,0);
  //private final SuperStructureState m_autoExtendState = new SuperStructureState(95, 95,0);
  private boolean needToSwitch = true;
  private boolean isDangerZone = true;
  private boolean shouldWait = true;
  /** Creates a new SmartMove. */
  public SmartMotion() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_carriage, m_elevator, m_telescop);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentState = RobotState.getInstance().getCurrentSuperStructureState();
    m_targetState = RobotState.getInstance().getTargetState();
    needToSwitch = !m_currentState.isInSameSide(m_targetState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.maybeShouldOpen();

    m_currentState = RobotState.getInstance().getCurrentSuperStructureState();


    needToSwitch = !m_currentState.isInSameSide(m_targetState);
    isDangerZone = m_currentState.getDegree() > -15 && m_currentState.getDegree() < 10;
    shouldWait = !(m_currentState.getAbsoluteHeight() > RobotState.getInstance().getEndEffector());

    SmartDashboard.putBoolean("switch", needToSwitch);
    SmartDashboard.putBoolean("zone", isDangerZone);
    SmartDashboard.putBoolean("shouldwait", shouldWait);



    if(needToSwitch || isDangerZone){
        m_elevator.setElevatorPosition(m_pivotState);
        m_telescop.setTelescopicPosition(m_pivotState);
      if(shouldWait){
        m_carriage.stop();
      }else {
        m_carriage.setCarriagePosition(m_targetState);
      }

    }else{
      
      if(RobotState.getInstance().getScoreTarget() == scoreLevel.HOMING || RobotState.getInstance().getScoreTarget() == scoreLevel.kStarting){
        m_elevator.setElevatorPosition(m_targetState);
        m_carriage.setCarriagePosition(m_targetState);
        m_telescop.setTelescopicPosition(m_targetState);
      }else{
        m_elevator.setElevatorPosition(m_targetState);
        m_carriage.setCarriagePosition(m_targetState);
        if(m_elevator.isAtSetpoint() && m_carriage.isAtSetpoint()){
          m_telescop.setTelescopicPosition(m_targetState);
        }
      }
      


    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*m_elevator.updateLastDemandedHeight(m_currentState.getHeight());
    m_carriage.updateLastDemandedRotation(m_currentState.getDegree());
    m_telescop.updateLastDemandedLength(m_currentState.getDegree());*/
    m_elevator.holdElevatorPosition();
    m_carriage.holdCarriagePosition();
    m_telescop.holdTelescopicPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_currentState.isAtDesiredState(m_targetState);
  }

  private double extensionAngle(){
    double absHeight = 90 - m_elevator.getHeight();
    double radian = Math.atan(absHeight / 60);
    return Math.toDegrees(radian) + 90;
  }

}
