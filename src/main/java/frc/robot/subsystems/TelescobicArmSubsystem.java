// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TelescobicArmConstants;
import frc.team6014.SuperStructureState;
import frc.team6014.lib.math.Gearbox;

public class TelescobicArmSubsystem extends SubsystemBase {

  private static TelescobicArmSubsystem mInstance;

  public static synchronized TelescobicArmSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new TelescobicArmSubsystem();
    }
    return mInstance;
  }

  private final TalonFX telescobicMaster = new TalonFX(TelescobicArmConstants.telesobicMotorID, Constants.CANIVORE_CANBUS);

  private final Gearbox falconGearbox = new Gearbox(1 * 40 * 10, 75 * 42 * 40);

  private Rotation2d currentRotation = new Rotation2d();

  private final PositionTorqueCurrentFOC m_torqueControl = new PositionTorqueCurrentFOC(0, 0, 0, false);

  public TelescobicArmSubsystem() {

    telescobicMaster.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = TelescobicArmConstants.kP;
    configs.Slot0.kI = TelescobicArmConstants.kI;
    configs.Slot0.kD = TelescobicArmConstants.kD;
    configs.Slot0.kS = TelescobicArmConstants.kS;
    configs.Slot0.kV = TelescobicArmConstants.kV;

    configs.Voltage.PeakForwardVoltage = TelescobicArmConstants.peakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = TelescobicArmConstants.peakReverseVoltage;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = TelescobicArmConstants.peakForwardTorqueCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = TelescobicArmConstants.peakReverseTorqueCurrent;

    configs.MotorOutput.NeutralMode = TelescobicArmConstants.neutralMode;
    configs.MotorOutput.Inverted = TelescobicArmConstants.invertedValue; // değiştir
    configs.MotorOutput.DutyCycleNeutralDeadband = TelescobicArmConstants.dutyCycleNeutralDeadband;

    configs.CurrentLimits.StatorCurrentLimit = TelescobicArmConstants.statorCurrentLimit;
    configs.CurrentLimits.StatorCurrentLimitEnable = TelescobicArmConstants.statorCurrentLimitEnable;
    configs.CurrentLimits.SupplyCurrentLimit = TelescobicArmConstants.supplyCurrentLimit;
    configs.CurrentLimits.SupplyCurrentLimitEnable = TelescobicArmConstants.supplyCurrentLimitEnable;

    telescobicMaster.getConfigurator().apply(configs);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = telescobicMaster.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }


  }

  @Override
  public void periodic() {
    update();
    SmartDashboard.putNumber("Falcon Degree", currentRotation.getDegrees());
    // This method will be called once per scheduler run
  }

  public void update() {
    var falconDegree = telescobicMaster.getRotorPosition();
    falconDegree.refresh();
    currentRotation = Rotation2d.fromDegrees((falconDegree.getValue() / falconGearbox.getRatio()) * 360);
  }

  public synchronized void setPosition(SuperStructureState state) {
    double targetDegree = state.getDegree();
    if (targetDegree >= 100) {
      targetDegree = 100;
    } else if (targetDegree <= -90) {
      targetDegree = -90;
    }
    double falconRotation = (targetDegree / 360) * falconGearbox.getRatio();
    telescobicMaster.setControl(m_torqueControl.withPosition(falconRotation));
  }

  public synchronized void setMotorOutput(double speed) {
    telescobicMaster.set(speed);
  }

  public synchronized void holdCurrentPosition(double lastdegrees) {
    double falconRotation = (currentRotation.getDegrees() / 360) * falconGearbox.getRatio();
    telescobicMaster.setControl(m_torqueControl.withPosition(falconRotation));
  }


  public Rotation2d getCurrentRotation() {
    return currentRotation;
  }
}
