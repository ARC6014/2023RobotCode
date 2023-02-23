// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.premiereCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.TelescobicArmSubsystem;
import frc.team6014.SuperStructureState;

public class HoldTelescop extends CommandBase {
  private final TelescobicArmSubsystem m_carriage = TelescobicArmSubsystem.getInstance();
  private SuperStructureState currenState;
  /** Creates a new HoldArm. */
  public HoldTelescop() {
    addRequirements(m_carriage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currenState = RobotState.getInstance().getCurrentSuperStructureState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_carriage.setLength(currenState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_carriage.setMotorOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
