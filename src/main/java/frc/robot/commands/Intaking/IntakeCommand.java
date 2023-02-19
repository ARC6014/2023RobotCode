// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intaking;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsytem;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  private IntakeSubsytem m_intake = IntakeSubsytem.getInstance();
  private GrabberSubsystem m_grabber = GrabberSubsystem.getInstance();
  private boolean m_isFinished = false;


  public IntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // numVal=0 : cube
    // numVal=1 : cone
   /*  if (RobotState.getInstance().getModeNum() == 1) {
      m_isFinished = true;

    }*/
    m_intake.extendIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.intakeCube();
    m_grabber.grab();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntake();
    m_intake.retractIntake();
    m_grabber.stop();
    
    m_isFinished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }

}
