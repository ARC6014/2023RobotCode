// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.RobotState.pieceState;
import frc.team6014.lib.drivers.AddressableLed;

public class SetLedState extends CommandBase {
  /** Creates a new SetLedState. */
  private final AddressableLed m_addressableLed;
  private final Timer m_timer = new Timer();

  public SetLedState(AddressableLed led) {
    m_addressableLed = led;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_addressableLed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("alo");
    if(RobotState.getInstance().getPiece() == pieceState.CONE){
      m_addressableLed.setLEDColor(Color.kRed);
    }if(RobotState.getInstance().getPiece() == pieceState.CUBE){
      m_addressableLed.setLEDColor(Color.kGreen);      
    }
    if(m_addressableLed.getTriggered()){
      
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
