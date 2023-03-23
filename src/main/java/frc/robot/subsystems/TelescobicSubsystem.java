// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.MotionMagicTorqueCurrentFOC;
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

public class TelescobicSubsystem extends SubsystemBase {

  public enum TelescopicControlState{
    HOMING,
    OPEN_LOOP,
    MOTION_MAGIC,
    TORQUE_CONTROL
  }

  private final TalonFX telescobicMaster = new TalonFX(TelescobicArmConstants.telesobicMotorID, Constants.CANIVORE_CANBUS);

  private final MotionMagicTorqueCurrentFOC m_motionMagic = new MotionMagicTorqueCurrentFOC(0,0,0, false);
  private final PositionTorqueCurrentFOC m_torqueControl = new PositionTorqueCurrentFOC(0,0,1,false); 
  private final DutyCycleOut m_percentOut = new DutyCycleOut(0, true, false);

  private final Gearbox falconGearbox = new Gearbox(1 * 18, 4 * 24);
  private final double pulleyCircumferenceInCM = Units.inchesToMeters(1.504) * Math.PI * 100;
  private double targetOutput = 0.0;
  private SuperStructureState targetState = new SuperStructureState();
  private double lastDemandedLength;

  public TelescopicControlState m_controlState = TelescopicControlState.OPEN_LOOP;

  private static TelescobicSubsystem mInstance;
  /** Creates a new TelescobicArmSubsystem. */
  public TelescobicSubsystem() {

    telescobicMaster.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 27;
    configs.Slot0.kI = 1.5;
    configs.Slot0.kD = 0.08;
    configs.Slot0.kS = 0.01;
    configs.Slot0.kV = 0;

    configs.Slot1.kP = 6;
    configs.Slot1.kI = 0.85;
    configs.Slot1.kD = 0.02;
    configs.Slot1.kS = 0.0;
    configs.Slot1.kV = 0;

    configs.Voltage.PeakForwardVoltage = 6;
    configs.Voltage.PeakReverseVoltage = -6;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 200;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 200;
    configs.MotionMagic.MotionMagicAcceleration = 470; // değiştir
    configs.MotionMagic.MotionMagicCruiseVelocity = 140; // değiştir
    configs.MotionMagic.MotionMagicJerk = 1050; //  değiştir

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // değiştir
    configs.MotorOutput.DutyCycleNeutralDeadband = 0.02;

    configs.CurrentLimits.StatorCurrentLimit = TelescobicArmConstants.statorCurrentLimit;
    configs.CurrentLimits.StatorCurrentLimitEnable = TelescobicArmConstants.statorCurrentLimitEnable;
    configs.CurrentLimits.SupplyCurrentLimit = TelescobicArmConstants.supplyCurrentLimit;
    configs.CurrentLimits.SupplyCurrentLimitEnable = TelescobicArmConstants.supplyCurrentLimitEnable;

    telescobicMaster.getConfigurator().apply(configs);

    lastDemandedLength = getLength();

  }

  public static TelescobicSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new TelescobicSubsystem();
    }
    return mInstance;
  }

  @Override
  public void periodic() {
    //maybeShouldStop();   

    //setTelescopicControlState(TelescopicControlState.HOMING);
   
    switch(m_controlState){
      case OPEN_LOOP:
        setMotorOutput();
        break;
      case MOTION_MAGIC:
        setLength();
        break;
      case TORQUE_CONTROL:
        holdPosition();
        break;
      default:
        stop();
        break;
    }



    SmartDashboard.putString("Telescopic State: ", m_controlState.toString());
    SmartDashboard.putNumber("Telescopic Length", getLength());
    SmartDashboard.putNumber("Telescopic Current", getCurrent());

    RobotState.getInstance().updateLength(getLength());
    // This method will be called once per scheduler run
  }

  public synchronized void setTelescopicOpenLoop(double output){
    if(m_controlState != TelescopicControlState.OPEN_LOOP){
      m_controlState =  TelescopicControlState.OPEN_LOOP;
    }
    targetOutput = output;
    lastDemandedLength = getLength();
  }

  public synchronized void setTelescopicPosition(SuperStructureState state){
    if(m_controlState != TelescopicControlState.MOTION_MAGIC){
      m_controlState =  TelescopicControlState.MOTION_MAGIC;
    }
    targetState = state;
    lastDemandedLength = getLength();
  }

  public synchronized void holdTelescopicPosition(){
    if(m_controlState != TelescopicControlState.TORQUE_CONTROL){
      m_controlState =  TelescopicControlState.TORQUE_CONTROL;
    }
  }

  public void setTelescopicControlState(TelescopicControlState state){
    m_controlState = state;
  }

  public void updateLastDemandedLength(double length){
    lastDemandedLength = length;
  }

  public void setMotorOutput(){
    telescobicMaster.setControl(m_percentOut.withOutput(targetOutput));
  }

  public void setLength(){
    double sprocketRotation = targetState.getLength() / pulleyCircumferenceInCM;
    telescobicMaster.setControl(m_motionMagic.withPosition(sprocketRotation * falconGearbox.getRatio()));
  }

  public void holdPosition(){
    double sprocketRotation = lastDemandedLength / pulleyCircumferenceInCM;
    telescobicMaster.setControl(m_torqueControl.withPosition(sprocketRotation * falconGearbox.getRatio()));
  }

  public void stop(){
    if(m_controlState != TelescopicControlState.HOMING){
      m_controlState =  TelescopicControlState.HOMING;
    }
    telescobicMaster.stopMotor();
  }

  public void overrideLength(double length){
    double pulleyRotation = (length  / pulleyCircumferenceInCM);
    telescobicMaster.setRotorPosition(pulleyRotation * falconGearbox.getRatio());
  }

  public void resetToZero(){
    overrideLength(92.225); //Min Length
  }

  public void resetToMax(){
    overrideLength(132.015); //Max Length
  }

  public double getLength(){
    var masterRot = telescobicMaster.getRotorPosition();
    masterRot.refresh(); 
    double pulleyRotation = masterRot.getValue() / falconGearbox.getRatio();
    return pulleyRotation * pulleyCircumferenceInCM;
    //return 92.225;
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
      holdPosition();
      System.out.println("Triggered Telescopic");
    }
  }

  public boolean isAtSetpoint(){
    return Math.abs(targetState.getLength() - getLength()) < 0.25; 
  } 


}
