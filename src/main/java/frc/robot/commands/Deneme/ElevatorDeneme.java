// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Deneme;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.team6014.SuperStructureState;

public class ElevatorDeneme extends CommandBase {
  private final ElevatorSubsystem m_elevator = ElevatorSubsystem.getInstance();
  private final SuperStructureState targetState = new SuperStructureState(124, 0, 0);
  private final DoubleSupplier joystick;
  private final BooleanSupplier m_button;
  private final BooleanSupplier m_secondButton;
  /** Creates a new ElevatorDeneme. */
  public ElevatorDeneme(DoubleSupplier output, BooleanSupplier button, BooleanSupplier secondButton) {
    joystick = output;
    m_button = button;
    m_secondButton = secondButton;
    addRequirements(m_elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   m_elevator.updateLastDemandedHeight(m_elevator.getHeight());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    if(joystick.getAsDouble() >= 0.04 || joystick.getAsDouble() <= -0.04){
      m_elevator.setElevatorOpenLoop(joystick.getAsDouble());
    }else if(m_button.getAsBoolean()){
      m_elevator.setElevatorPosition(targetState);
    }else{
      m_elevator.holdElevatorPosition();
    }
     /* 
    if(m_secondButton.getAsBoolean()){
      m_elevator.holdElevatorPosition();
    }else if(m_button.getAsBoolean()){
      m_elevator.setElevatorPosition(targetState);
    }else{
      m_elevator.setElevatorOpenLoop(joystick.getAsDouble());
    }*/
    
  /*   System.out.println(
    RobotState.getInstance().getCurrentSuperStructureState().getHeight() );*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
