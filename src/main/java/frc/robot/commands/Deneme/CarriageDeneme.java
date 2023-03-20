// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Deneme;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TelescobicSubsystem;
import frc.team6014.SuperStructureState;

public class CarriageDeneme extends CommandBase {
  private final CarriageSubsystem m_carriage = CarriageSubsystem.getInstance();
  private final SuperStructureState targetState = new SuperStructureState(124, 0, 90);
  private final DoubleSupplier joystick;
  private final BooleanSupplier m_button;
  private final BooleanSupplier m_secondButton;
  /** Creates a new ElevatorDeneme. */
  public CarriageDeneme(DoubleSupplier output, BooleanSupplier button, BooleanSupplier secondButton) {
    joystick = output;
    m_button = button;
    m_secondButton = secondButton;
    addRequirements(m_carriage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_carriage.updateLastDemandedRotation(m_carriage.getRotation());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     if(joystick.getAsDouble() >= 0.04 || joystick.getAsDouble() <= -0.04){
      m_carriage.setCarriageOpenLoop(joystick.getAsDouble());
    }else if(m_button.getAsBoolean()){
      m_carriage.setCarriagePosition(targetState);
    }else{
      m_carriage.holdCarriagePosition();
    }
    ElevatorSubsystem.getInstance().setElevatorPosition(targetState);
    //TelescobicSubsystem.getInstance().holdTelescopicPosition();/* 
    /*if(m_secondButton.getAsBoolean()){
      m_carriage.holdCarriagePosition();
    }else if(m_button.getAsBoolean()){
      m_carriage.setCarriagePosition(targetState);
    }else{
      m_carriage.setCarriageOpenLoop(joystick.getAsDouble());
    }*/
    
  /*   System.out.println(
    RobotState.getInstance().getCurrentSuperStructureState().getHeight() );*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_carriage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
