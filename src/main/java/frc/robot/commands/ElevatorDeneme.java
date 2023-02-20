// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TelescobicArmSubsystem;
import frc.team6014.SuperStructureState;

public class ElevatorDeneme extends CommandBase {
  private final TelescobicArmSubsystem m_carraige = TelescobicArmSubsystem.getInstance();
  private final double targetHeight = 100;
  private final DoubleSupplier joystick;
  private final BooleanSupplier m_button;
  private final BooleanSupplier m_secondButton;
  /** Creates a new ElevatorDeneme. */
  public ElevatorDeneme(DoubleSupplier output, BooleanSupplier button, BooleanSupplier secondButton) {
    joystick = output;
    m_button = button;
    m_secondButton = secondButton;
    addRequirements(m_carraige);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_button.getAsBoolean()){
      m_carraige.setMotorOutput(joystick.getAsDouble());
    }else{
      m_carraige.setLength(new SuperStructureState(0, targetHeight , 0));
    }
  /*   System.out.println(
    RobotState.getInstance().getCurrentSuperStructureState().getHeight() );*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_carraige.setMotorOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
