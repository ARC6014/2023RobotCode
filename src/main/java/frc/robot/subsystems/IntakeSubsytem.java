// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;

public class IntakeSubsytem extends SubsystemBase {
  /** Creates a new IntakeSubsytem. */
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless);

  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.IntakeConstants.intakeDoubleSolenoidPort1, Constants.IntakeConstants.intakeDoubleSolenoidPort2);

  private final SparkMaxPIDController m_PID = intakeMotor.getPIDController();

  private final double m_rpm;


  private static IntakeSubsytem m_instance;

  public IntakeSubsytem() {

    intakeMotor.setInverted(Constants.IntakeConstants.isIntakeInverted);
    intakeMotor.setIdleMode(Constants.IntakeConstants.idleMode);

    m_PID.setP(Constants.IntakeConstants.kP);
    m_PID.setI(Constants.IntakeConstants.kI);
    m_PID.setD(Constants.IntakeConstants.kD);

    // TODO: Check the last parameter (RPM)
    intakeMotor.setSmartCurrentLimit(Constants.IntakeConstants.stallCurrentLimit, Constants.IntakeConstants.freeCurrentLimit, (int) Constants.IntakeConstants.RPM);

    m_rpm = Constants.IntakeConstants.RPM;
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
    // This method will be called once per scheduler run
    // @Log.BooleanBox(name="Intake Open?")
    //isIntakeOpen();

  }

  /*@Log.BooleanBox(name = "Intake Open?", rowIndex = 0, columnIndex = 0)
  public boolean isIntakeOpen() {
    if (intakeSolenoid.isRevSolenoidDisabled()) {
      return false;
    } else {
      return true;
    }
  }*/

  public boolean intakeAtSetPoint() {
    if(Math.abs(m_rpm-getRPM()) < Constants.IntakeConstants.kIntakeTolerance) {
      return true;
    }  
    return false;
  }

  private double getRPM() {
    return intakeMotor.getEncoder().getVelocity();
  }

  public void extendIntake() {
    intakeSolenoid.set(Value.kForward);
  }

  public void retractIntake() {
    intakeSolenoid.set(Value.kReverse);
  }

  public void getCurrent() {
    intakeMotor.getOutputCurrent();
  }

  public void intakeCube() {
    intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
  }

  public void releaseCube() {
    intakeMotor.set(Constants.IntakeConstants.outtakeSpeed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }
}
