// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenixpro.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
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
  private final Timer m_timer = new Timer();

  private final MotionMagicTorqueCurrentFOC m_motionMagic = new MotionMagicTorqueCurrentFOC(0,0,0, false);
  private final PositionTorqueCurrentFOC m_torqueControl = new PositionTorqueCurrentFOC(0,0,1,false); 
  private final DutyCycleOut m_percentOut = new DutyCycleOut(0, true, false);

  private final Gearbox falconGearbox = new Gearbox(1 * 42 * 18, 80 * 54 * 40);
  private double targetOutput = 0.0;
  private SuperStructureState targetState = new SuperStructureState();
  private double lastDemandedRotation;
  private double lastAbsoluteTime;
  private boolean isSlow = false; 

  public CarriageControlState m_controlState = CarriageControlState.OPEN_LOOP;

  private static CarriageSubsystem m_instance;

  public CarriageSubsystem() {

    carriageMaster.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 11.5;
    configs.Slot0.kI = 0.000;
    configs.Slot0.kD = 1.12;
    configs.Slot0.kS = 0.005;
    configs.Slot0.kV = 0.000;

    configs.Slot1.kP = 8;
    configs.Slot1.kI = 0.0;
    configs.Slot1.kD = 3.5;
    configs.Slot1.kS = 0.001;
    configs.Slot1.kV = 0.00;

    configs.Voltage.PeakForwardVoltage = 10;
    configs.Voltage.PeakReverseVoltage = -10;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 180;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 180;;
    configs.MotionMagic.MotionMagicAcceleration = 260; // değiştir
    configs.MotionMagic.MotionMagicCruiseVelocity = 85; // değiştir
    configs.MotionMagic.MotionMagicJerk = 600; //  değiştir
    
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted = CarriageConstants.invertedValue; 
    configs.MotorOutput.DutyCycleNeutralDeadband = CarriageConstants.dutyCycleNeutralDeadband;
    configs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.100;

    configs.CurrentLimits.StatorCurrentLimit = CarriageConstants.statorCurrentLimit;
    configs.CurrentLimits.StatorCurrentLimitEnable = CarriageConstants.statorCurrentLimitEnable;
    configs.CurrentLimits.SupplyCurrentLimit = CarriageConstants.supplyCurrentLimit;
    configs.CurrentLimits.SupplyCurrentLimitEnable = CarriageConstants.supplyCurrentLimitEnable;

    carriageMaster.getConfigurator().apply(configs);

    m_timer.reset();
    m_timer.start();
    m_encoder.reset();

    lastDemandedRotation = getRotation();
    lastAbsoluteTime = m_timer.get();

    new InstantCommand(() -> m_encoder.setPositionOffset(0.1), this);
    m_encoder.setPositionOffset(0.055);

    resetToAbsolute();
  }

  public static CarriageSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new CarriageSubsystem();
    }
    return m_instance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Carriage Encoder: ", getAbsolutePosition());
    SmartDashboard.putNumber("Carriage Falcon Degree", getRotation());
    SmartDashboard.putNumber("Carriage Current", getCurrent());

    SmartDashboard.putNumber("ofsett", m_encoder.getPositionOffset());
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

    autoCalibration();

    
    SmartDashboard.putString("Carriage State: ", m_controlState.toString());

    RobotState.getInstance().updateDegree(getRotation());
    //System.out.println(RobotState.getInstance().getCurrentSuperStructureState().getAbsoluteHeight());
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

  public synchronized void updateLastDemandedRotation(double rotation){
    lastDemandedRotation = rotation;
  }

  public void setCarriageControlState(CarriageControlState state){
    m_controlState = state;
  }

  public void setMotorOutput(){
    carriageMaster.setControl(m_percentOut.withOutput(targetOutput));
  }

  public void setMotorOutput(double output) {
    carriageMaster.setControl(m_percentOut.withOutput(output));
  }

  public void setRotation(){
    if(getRotation()>=100){
      stop();
    }else{
      double sprocketRotation = targetState.getDegree() / 360;
      carriageMaster.setControl(m_motionMagic.withPosition(sprocketRotation * falconGearbox.getRatio()));
    }

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

  public void slowCarriage(){
    if(isSlow == true) return;
    MotionMagicConfigs need = new MotionMagicConfigs();
    need.MotionMagicAcceleration = 65; // değiştir
    need.MotionMagicCruiseVelocity = 25; // değiştir
    need.MotionMagicJerk = 220; //  değiştir
    carriageMaster.getConfigurator().apply(need);
    isSlow = true;
  }

  public void fastCarriage(){
    if(isSlow == false) return;
    MotionMagicConfigs need = new MotionMagicConfigs();
    need.MotionMagicAcceleration = 370; // değiştir
    need.MotionMagicCruiseVelocity = 120; // değiştir
    need.MotionMagicJerk = 750; //  değiştir
    carriageMaster.getConfigurator().apply(need);
    isSlow = false;
  }

  public void resetToAbsolute() {
    double position = getAbsolutePosition();
    carriageMaster.setRotorPosition((position / 360) * falconGearbox.getRatio());
    lastAbsoluteTime = m_timer.get();
  }

  public double getAbsolutePosition() {
    if(m_encoder.getAbsolutePosition() > 0 && m_encoder.getAbsolutePosition() < 0.6){
      return m_encoder.getAbsolutePosition() * -360 + 226;
    }
    return m_encoder.getAbsolutePosition() * -360 + 226;



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

  public void autoCalibration(){
    if( (m_timer.get() - lastAbsoluteTime) > 0.2  && (Math.abs(getAbsolutePosition() - getRotation()) >= 1.2) ) {
    resetToAbsolute();
    lastAbsoluteTime = m_timer.get();
    }
  }

  public void maybeShouldStop(){
    var currentVel = carriageMaster.getRotorVelocity();
    currentVel.refresh();
    if(Util.epsilonEquals(currentVel.getValue() / falconGearbox.getRatio(), 0 , 0.1) && getCurrent() >= 100)
    {//Kalibre ET!!!
      holdPosition();
      System.out.println("Triggered Carriage");
    }
  }

  public boolean isAtSetpoint(){
    return Math.abs(targetState.getDegree() - getRotation()) <= 3;
  }

}
