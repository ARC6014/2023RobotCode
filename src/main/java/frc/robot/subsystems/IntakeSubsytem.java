// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsytem extends SubsystemBase {
  /** Creates a new IntakeSubsytem. */
  private final WPI_TalonSRX intakeMaster = new WPI_TalonSRX(30);

  private static IntakeSubsytem m_instance;
  
  public IntakeSubsytem() {

    intakeMaster.setInverted(false);
    intakeMaster.setNeutralMode(NeutralMode.Coast);
  }

  public static IntakeSubsytem getInstance(){
    if(m_instance == null){
        m_instance = new IntakeSubsytem();
    }
    return m_instance;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeBall(){
    intakeMaster.set(ControlMode.PercentOutput, 0.5);
  }

  public void releaseBall(){
    intakeMaster.set(ControlMode.PercentOutput, -0.5); 
  }

  public void stopIntake(){
    intakeMaster.setVoltage(0.0);
  }
}
