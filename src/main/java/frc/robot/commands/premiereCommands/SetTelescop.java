// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.premiereCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescobicArmSubsystem;
import frc.team6014.SuperStructureState;

public class SetTelescop extends CommandBase {
  private final TelescobicArmSubsystem m_telescobicArmSubsystem = TelescobicArmSubsystem.getInstance();
  private SuperStructureState m_targetState;
  /** Creates a new SetTelescop. */
  public SetTelescop(SuperStructureState state) {
    m_targetState = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_telescobicArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_telescobicArmSubsystem.setLength(m_targetState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_telescobicArmSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_telescobicArmSubsystem.isAtSetpoint(m_targetState);
  }
}
