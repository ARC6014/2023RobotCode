// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotState.scoreLevel;

public class IntakeSubsytem extends SubsystemBase {
  /** Creates a new IntakeSubsytem. */
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorID);

  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      IntakeConstants.intakeDoubleSolenoidPort1, IntakeConstants.intakeDoubleSolenoidPort2);


  private static IntakeSubsytem m_instance;

  public IntakeSubsytem() {

    intakeMotor.setInverted(Constants.IntakeConstants.isIntakeInverted);
    intakeMotor.setNeutralMode(NeutralMode.Brake);

  }

  public static IntakeSubsytem getInstance() {
    if (m_instance == null) {
      m_instance = new IntakeSubsytem();
    }
    return m_instance;
  }

  @Override
  public void periodic() {

  }

  public void extendIntake() {
    intakeSolenoid.set(Value.kForward);
  }

  public void retractIntake() {
    intakeSolenoid.set(Value.kReverse);
  }

  public void intakeOff(){
    intakeSolenoid.set(Value.kOff);
  }

  public void intakeCube() {
    intakeMotor.set(ControlMode.PercentOutput, 0.4);
  }

  public void releaseCube() {
    intakeMotor.set(ControlMode.PercentOutput, -0.4);
  }

  public void stop() {
    intakeMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public double getCurrent(){
    return intakeMotor.getStatorCurrent();
  }

  public void maybeShouldOpen(){
    double a = RobotState.getInstance().getScoreTarget() == scoreLevel.Intake? -15 : -17.5;
    if(CarriageSubsystem.getInstance().getRotation() < a){
      extendIntake();
    }else{
      retractIntake();
    }
  }
}
