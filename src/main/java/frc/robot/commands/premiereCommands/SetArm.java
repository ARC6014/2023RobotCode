// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.premiereCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CarriageSubsystem;
import frc.team6014.SuperStructureState;

public class SetArm extends CommandBase {
  private final CarriageSubsystem m_carraige = CarriageSubsystem.getInstance();
  private SuperStructureState m_targetState;
  /** Creates a new SetArm. */
  public SetArm(SuperStructureState state) {
    m_targetState = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_carraige);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_carraige.setPosition(m_targetState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_carraige.setMotorOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_carraige.isAtSetpoint(m_targetState);
  }
}
