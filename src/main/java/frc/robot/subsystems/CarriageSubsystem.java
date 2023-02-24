// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenixpro.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CarriageConstants;
import frc.team6014.SuperStructureState;
import frc.team6014.lib.math.Gearbox;
import frc.team6014.lib.util.Util;

public class CarriageSubsystem extends SubsystemBase {

  public enum CarriageControlState{
    HOMING,
    OPEN_LOOP,
    MOTION_MAGIC,
    TORQUE_CONTROL
  }

  private final TalonFX carriageMaster = new TalonFX(CarriageConstants.carriageMasterID, Constants.CANIVORE_CANBUS);
  private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(CarriageConstants.encoderID);

  private final MotionMagicTorqueCurrentFOC m_motionMagic = new MotionMagicTorqueCurrentFOC(0,0,0, false);
  private final PositionTorqueCurrentFOC m_torqueControl = new PositionTorqueCurrentFOC(0,0,1,false); 
  private final DutyCycleOut m_percentOut = new DutyCycleOut(0, true, false);

  private final Gearbox falconGearbox = new Gearbox(1 * 40 * 10, 75 * 42 * 40);
  private final Gearbox encoderGearbox = new Gearbox(CarriageConstants.encoderDrivingGear,
      CarriageConstants.encoderDrivenGear);
  private double targetOutput = 0.0;
  private SuperStructureState targetState = new SuperStructureState();
  private double lastDemandedRotation;

  public CarriageControlState m_controlState = CarriageControlState.OPEN_LOOP;

  private static CarriageSubsystem m_instance;

  public CarriageSubsystem() {

    carriageMaster.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 3.5;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.0;
    configs.Slot0.kS = 0.1;
    configs.Slot0.kV = 0.0;

    configs.Slot1.kP = 0.0;
    configs.Slot1.kI = 0.0;
    configs.Slot1.kD = 0.0;
    configs.Slot1.kS = 0.0;
    configs.Slot1.kV = 0.0;

    configs.Voltage.PeakForwardVoltage = CarriageConstants.peakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = CarriageConstants.peakReverseVoltage;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = CarriageConstants.peakForwardTorqueCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = CarriageConstants.peakReverseTorqueCurrent;
    configs.MotionMagic.MotionMagicAcceleration = 150; // değiştir
    configs.MotionMagic.MotionMagicCruiseVelocity = 100; // değiştir
    configs.MotionMagic.MotionMagicJerk = 160; //  değiştir
    
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted = CarriageConstants.invertedValue; 
    configs.MotorOutput.DutyCycleNeutralDeadband = CarriageConstants.dutyCycleNeutralDeadband;

    configs.CurrentLimits.StatorCurrentLimit = CarriageConstants.statorCurrentLimit;
    configs.CurrentLimits.StatorCurrentLimitEnable = CarriageConstants.statorCurrentLimitEnable;
    configs.CurrentLimits.SupplyCurrentLimit = CarriageConstants.supplyCurrentLimit;
    configs.CurrentLimits.SupplyCurrentLimitEnable = CarriageConstants.supplyCurrentLimitEnable;

    carriageMaster.getConfigurator().apply(configs);

    m_encoder.reset();

    lastDemandedRotation = getRotation();
  }

  public static CarriageSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new CarriageSubsystem();
    }
    return m_instance;
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Carriage Encoder: ", getAbsolutePosition());
    //SmartDashboard.putNumber("Carriage Falcon Degree", getRotation());
    //SmartDashboard.putNumber("Carriage Current", getCurrent());

    switch(m_controlState){
      case OPEN_LOOP:
        setMotorOutput();
        break;
      case MOTION_MAGIC:
        setRotation();
        break;
      case TORQUE_CONTROL:
        holdPosition();
        break;
      default:
        stop();
        break;
    }

    SmartDashboard.putString("Carriage State: ", m_controlState.toString());

    //RobotState.getInstance().updateDegree(getRotation());
    // This method will be called once per scheduler run
  }

  public synchronized void setCarriageOpenLoop(double output){
    if(m_controlState != CarriageControlState.OPEN_LOOP){
      m_controlState =  CarriageControlState.OPEN_LOOP;
    }
    targetOutput = output;
    lastDemandedRotation = getRotation();
  }

  public synchronized void setCarriagePosition(SuperStructureState state){
    if(m_controlState != CarriageControlState.MOTION_MAGIC){
      m_controlState =  CarriageControlState.MOTION_MAGIC;
    }
    targetState = state;
    lastDemandedRotation = getRotation();
  }

  public synchronized void holdCarriagePosition(){
    if(m_controlState != CarriageControlState.TORQUE_CONTROL){
      m_controlState =  CarriageControlState.TORQUE_CONTROL;
    }
  }

  public void setCarriageControlState(CarriageControlState state){
    m_controlState = state;
  }

  public void setMotorOutput(){
    carriageMaster.setControl(m_percentOut.withOutput(targetOutput));
  }

  public void setRotation(){
    double targetDegree = targetState.getDegree();
    if (targetDegree >= 85) {// max angle
      targetDegree = 85;
    } else if (targetDegree <= -45) { // min angle
      targetDegree = -45;
    }
    double sprocketRotation = targetState.getDegree() / 360;
    carriageMaster.setControl(m_motionMagic.withPosition(sprocketRotation * falconGearbox.getRatio()));
  }

  public void holdPosition(){
    double sprocketRotation = lastDemandedRotation / 360;
    carriageMaster.setControl(m_torqueControl.withPosition(sprocketRotation * falconGearbox.getRatio()));
  }

  public void stop(){
    if(m_controlState != CarriageControlState.HOMING){
      m_controlState =  CarriageControlState.HOMING;
    }
    carriageMaster.stopMotor();
  }

  public void resetToAbsolute() {
    double position = getAbsolutePosition();
    carriageMaster.setRotorPosition((position / 360) * falconGearbox.getRatio());
  }

  public double getAbsolutePosition() {
    return (((m_encoder.getAbsolutePosition() * 360) / encoderGearbox.getRatio() * -1) + 89.57 - 0.8);
  }

  public double getRotation() {
    var falconDegree = carriageMaster.getRotorPosition();
    falconDegree.refresh();
    return falconDegree.getValue() * 360 / falconGearbox.getRatio();
  }

  public double getCurrent(){
    var masterCurrent = carriageMaster.getStatorCurrent();
    masterCurrent.refresh();
    return masterCurrent.getValue();
  }

  public void maybeShouldStop(){
    var currentVel = carriageMaster.getRotorVelocity();
    currentVel.refresh();
    if(Util.epsilonEquals(currentVel.getValue() / falconGearbox.getRatio(), 0 , 0.1) && getCurrent() >= 100){//Kalibre ET!!!
      holdPosition();
      System.out.println("Triggered Carriage");
    }
  }

  public boolean isAtSetpoint(){
    return Math.abs(targetState.getDegree() - getRotation()) <= 3;
  }

}
