// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TelescobicArmSubsystem;
import frc.team6014.SuperStructureState;
import frc.team6014.lib.util.TunableNumber;

public class SuperStructureCalibration extends CommandBase {
  private final CarriageSubsystem m_carriage = CarriageSubsystem.getInstance();
  private final ElevatorSubsystem m_elevator = ElevatorSubsystem.getInstance();
  private final TelescobicArmSubsystem m_telescobic = TelescobicArmSubsystem.getInstance();

  private SuperStructureState targetState = new SuperStructureState(0, 0, 0);
  /** Creates a new SuperStructureCalibration. */
  public SuperStructureCalibration() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_carriage, m_elevator, m_telescobic);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Target Height", targetState.getHeight());

    SmartDashboard.putNumber("CURRENT Height ", RobotState.getInstance().getCurrentSuperStructureState().getHeight());

    SmartDashboard.putNumber("Target Degree", targetState.getDegree());

    SmartDashboard.putNumber("CURRENT Degree ", RobotState.getInstance().getCurrentSuperStructureState().getDegree());

    SmartDashboard.putNumber("Target Length", targetState.getLength());

    SmartDashboard.putNumber("CURRENT Length ", RobotState.getInstance().getCurrentSuperStructureState().getLength());

    SmartDashboard.putBoolean("At Desired State: ", targetState.isAtDesiredState(RobotState.getInstance().getCurrentSuperStructureState()));

    SmartDashboard.putNumber("abs height", RobotState.getInstance().getCurrentSuperStructureState().getAbsoluteHeight());

    m_carriage.setPosition(targetState);
    m_elevator.setHeight(targetState);
    m_telescobic.setLength(targetState);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_carriage.setMotorOutput(0);
    m_telescobic.stop();
    m_elevator.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
