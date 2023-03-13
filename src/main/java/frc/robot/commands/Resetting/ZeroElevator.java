// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Resetting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroElevator extends CommandBase {
  /** Creates a new ZeroElevator. */
  private final ElevatorSubsystem m_elevator = ElevatorSubsystem.getInstance();
  private Timer m_timer = new Timer();
  private boolean m_isFinished;

  public ZeroElevator() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_isFinished = false;
    m_elevator.setElevatorOpenLoop(-0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() > 0.5 && m_elevator.getCurrent() > 20) {
      m_elevator.stop();
      m_elevator.resetToZero();
      m_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_elevator.updateLastDemandedHeight(45);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
