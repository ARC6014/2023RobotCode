// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class IntakeSubsytem extends SubsystemBase {
  /** Creates a new IntakeSubsytem. */
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless);

  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.IntakeConstants.intakeDoubleSolenoidPort1, Constants.IntakeConstants.intakeDoubleSolenoidPort2);


  private static IntakeSubsytem m_instance;

  public IntakeSubsytem() {

    intakeMotor.setInverted(Constants.IntakeConstants.isIntakeInverted);
    intakeMotor.setIdleMode(Constants.IntakeConstants.idleMode);

    intakeMotor.setSmartCurrentLimit(Constants.IntakeConstants.stallCurrentLimit, Constants.IntakeConstants.freeCurrentLimit, (int) Constants.IntakeConstants.RPM);

    intakeMotor.burnFlash();

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
    intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
  }

  public void releaseCube() {
    intakeMotor.set(Constants.IntakeConstants.outtakeSpeed);
  }

  public void stop() {
    intakeMotor.stopMotor();
  }

  public double getCurrent(){
    return intakeMotor.getOutputCurrent();
  }
}
