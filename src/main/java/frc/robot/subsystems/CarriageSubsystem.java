// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CarriageConstants;
import frc.team6014.SuperStructureState;
import frc.team6014.lib.math.Gearbox;

public class CarriageSubsystem extends SubsystemBase {

  private static CarriageSubsystem mInstance;

  public static synchronized CarriageSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new CarriageSubsystem();
    }
    return mInstance;
  }

  private final TalonFX carriageMaster = new TalonFX(CarriageConstants.carriageMasterID, Constants.CANIVORE_CANBUS);
  private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(CarriageConstants.encoderID);

  private final Gearbox falconGearbox = new Gearbox(1 * 40 * 10, 75 * 42 * 40);
  private final Gearbox encoderGearbox = new Gearbox(CarriageConstants.encoderDrivingGear,
      CarriageConstants.encoderDrivenGear);

  private final PositionTorqueCurrentFOC m_torqueControl = new PositionTorqueCurrentFOC(0, 0, 0, false);

  public CarriageSubsystem() {

    carriageMaster.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = CarriageConstants.kP;
    configs.Slot0.kI = CarriageConstants.kI;
    configs.Slot0.kD = CarriageConstants.kD;
    configs.Slot0.kS = CarriageConstants.kS;
    configs.Slot0.kV = CarriageConstants.kV;

    configs.Voltage.PeakForwardVoltage = CarriageConstants.peakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = CarriageConstants.peakReverseVoltage;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = CarriageConstants.peakForwardTorqueCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = CarriageConstants.peakReverseTorqueCurrent;

    configs.MotorOutput.NeutralMode = CarriageConstants.neutralMode;
    configs.MotorOutput.Inverted = CarriageConstants.invertedValue; 
    configs.MotorOutput.DutyCycleNeutralDeadband = CarriageConstants.dutyCycleNeutralDeadband;

    configs.CurrentLimits.StatorCurrentLimit = CarriageConstants.statorCurrentLimit;
    configs.CurrentLimits.StatorCurrentLimitEnable = CarriageConstants.statorCurrentLimitEnable;
    configs.CurrentLimits.SupplyCurrentLimit = CarriageConstants.supplyCurrentLimit;
    configs.CurrentLimits.SupplyCurrentLimitEnable = CarriageConstants.supplyCurrentLimitEnable;

    carriageMaster.getConfigurator().apply(configs);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = carriageMaster.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    m_encoder.reset();

    resetToAbsolute();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder: ", getAbsolutePosition());
    SmartDashboard.putNumber("Falcon Degree", getCurrentRotation());
    // This method will be called once per scheduler run
  }

  public double getCurrentRotation() {
    var falconDegree = carriageMaster.getRotorPosition();
    falconDegree.refresh();
    return falconDegree.getValue() * 360 / falconGearbox.getRatio();
  }

  public synchronized void setPosition(SuperStructureState state) {
    double targetDegree = state.getDegree();
    if (targetDegree >= 110) {
      targetDegree = 110;
    } else if (targetDegree <= -90) {
      targetDegree = -90;
    }
    double falconRotation = (targetDegree / 360) * falconGearbox.getRatio();
    carriageMaster.setControl(m_torqueControl.withPosition(falconRotation));
  }

  public synchronized void setMotorOutput(double speed) {
    carriageMaster.set(speed);
  }

  public synchronized void resetToAbsolute() {
    double position = getAbsolutePosition();
    carriageMaster.setRotorPosition((position / 360) * falconGearbox.getRatio());
  }

  public synchronized double getAbsolutePosition() {
    return ((Math.toDegrees(m_encoder.getAbsolutePosition() * 2 * Math.PI) / encoderGearbox.getRatio() * -1) + 88);
  }

}
