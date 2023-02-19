// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.TelescobicArmConstants;
import frc.team6014.SuperStructureState;
import frc.team6014.lib.math.Gearbox;
import frc.team6014.lib.util.Util;

public class TelescobicArmSubsystem extends SubsystemBase {

  private static TelescobicArmSubsystem mInstance;

  public static synchronized TelescobicArmSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new TelescobicArmSubsystem();
    }
    return mInstance;
  }

  private final TalonFX telescobicMaster = new TalonFX(TelescobicArmConstants.telesobicMotorID, Constants.CANIVORE_CANBUS);
  private final Gearbox falconGearbox = new Gearbox(1 * 18, 4 * 24);
  private final double pulleyCircumferenceInCM = Units.inchesToMeters(1.504) * Math.PI * 100;
  private final PositionTorqueCurrentFOC m_torqueControl = new PositionTorqueCurrentFOC(0, 0, 0, false);
  private double lastTelescopicCommandLength;

  public TelescobicArmSubsystem() {

    telescobicMaster.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 0;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 0;
    configs.Slot0.kS = 0;
    configs.Slot0.kV = 0;

    configs.Voltage.PeakForwardVoltage = 4;
    configs.Voltage.PeakReverseVoltage = -4;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 200;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 200;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // değiştir
    configs.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    configs.CurrentLimits.StatorCurrentLimit = TelescobicArmConstants.statorCurrentLimit;
    configs.CurrentLimits.StatorCurrentLimitEnable = TelescobicArmConstants.statorCurrentLimitEnable;
    configs.CurrentLimits.SupplyCurrentLimit = TelescobicArmConstants.supplyCurrentLimit;
    configs.CurrentLimits.SupplyCurrentLimitEnable = TelescobicArmConstants.supplyCurrentLimitEnable;

    telescobicMaster.getConfigurator().apply(configs);


  }

  @Override
  public void periodic() {
    //maybeShouldStop();   
    SmartDashboard.putNumber("Telescopic Length: ", getLength());
    SmartDashboard.putNumber("Telescopic Current: ", getCurrent());

    RobotState.getInstance().updateLength(getLength());
     // This method will be called once per scheduler run
  }

  public synchronized void setMotorOutput(double output){
    telescobicMaster.set(output);
    lastTelescopicCommandLength = getLength();
  }

  public synchronized void setLength(SuperStructureState state){
    double pulleyRotation = state.getLength() / pulleyCircumferenceInCM;
    telescobicMaster.setControl(m_torqueControl.withPosition(pulleyRotation * falconGearbox.getRatio()));
    lastTelescopicCommandLength = getLength();
  }

  public synchronized void holdCurrentPosition(){
    telescobicMaster.setControl(m_torqueControl.withPosition(lastTelescopicCommandLength));
  }

  public void stop(){
    telescobicMaster.set(0.0);
  }

  public void overrideLength(double length){
    double pulleyRotation = (length  / pulleyCircumferenceInCM);
    telescobicMaster.setRotorPosition(pulleyRotation * falconGearbox.getRatio());
  }

  public void resetToZero(){
    overrideLength(92.225); //Min Height
  }

  public void resetToMax(){
    overrideLength(132.015); //Max Height
  }

  public double getLength(){
    var masterRot = telescobicMaster.getRotorPosition();
    masterRot.refresh(); 
    double pulleyRotation = masterRot.getValue() / falconGearbox.getRatio();
    return pulleyRotation * pulleyCircumferenceInCM;
  }

  public double getCurrent(){
    var masterCurrent = telescobicMaster.getStatorCurrent();
    masterCurrent.refresh();
    return masterCurrent.getValue();
  }

  public void maybeShouldStop(){
    var currentVel = telescobicMaster.getRotorVelocity();
    currentVel.refresh();
    if(Util.epsilonEquals(currentVel.getValue() / falconGearbox.getRatio(), 0 , 0.1) && getCurrent() >= 100){//Kalibre ET!!!
      telescobicMaster.setControl(m_torqueControl.withPosition(lastTelescopicCommandLength - 0.005));
    }
  }

  public boolean isAtSetpoint(SuperStructureState state){
    return Math.abs(state.getLength() - getLength()) < 0.5; 
  } 


}
