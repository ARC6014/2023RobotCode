// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Deneme;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescobicSubsystem;
import frc.team6014.SuperStructureState;

public class TelescopicDeneme extends CommandBase {
  private final TelescobicSubsystem m_telescop = TelescobicSubsystem.getInstance();
  private final SuperStructureState targetState = new SuperStructureState(0, 110, 0);
  private final DoubleSupplier joystick;
  private final BooleanSupplier m_button;
  private final BooleanSupplier m_secondButton;
  /** Creates a new ElevatorDeneme. */
  public TelescopicDeneme(DoubleSupplier output, BooleanSupplier button, BooleanSupplier secondButton) {
    joystick = output;
    m_button = button;
    m_secondButton = secondButton;
    addRequirements(m_telescop);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
/*     if(joystick.getAsDouble() >= 0.04 || joystick.getAsDouble() <= -0.04){
      m_telescop.setTelescopicOpenLoop(joystick.getAsDouble());
    }else if(m_button.getAsBoolean()){
      m_telescop.setTelescopicPosition(targetState);
    }else{
      m_telescop.holdTelescopicPosition();
    }*/
 
    if(m_secondButton.getAsBoolean()){
      m_telescop.holdTelescopicPosition();
    }else if(m_button.getAsBoolean()){
      m_telescop.setTelescopicPosition(targetState);
    }else{
      m_telescop.setTelescopicOpenLoop(joystick.getAsDouble());
    }
    
  /*   System.out.println(
    RobotState.getInstance().getCurrentSuperStructureState().getHeight() );*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_telescop.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
