// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Resetting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescobicSubsystem;

public class ZeroTelescopic extends CommandBase {
  /** Creates a new ZeroTelescopic. */
  private final TelescobicSubsystem m_telescobicArmSubsystem = TelescobicSubsystem.getInstance();
  private Timer m_timer = new Timer();
  private boolean m_isFinished;

  public ZeroTelescopic() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_telescobicArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_isFinished = false;
    System.out.println("alo");
  //  m_telescobicArmSubsystem.setMotorOutput(-0.1);
  m_telescobicArmSubsystem.setTelescopicOpenLoop(-0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() > 0.5 && m_telescobicArmSubsystem.getCurrent() > 18) {
      m_telescobicArmSubsystem.stop();
      m_telescobicArmSubsystem.resetToZero();
      m_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_telescobicArmSubsystem.updateLastDemandedLength(m_telescobicArmSubsystem.getLength() + 0.1);
    m_telescobicArmSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}


