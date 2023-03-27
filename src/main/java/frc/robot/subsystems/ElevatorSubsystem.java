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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.team6014.SuperStructureState;
import frc.team6014.lib.drivers.LimitSwitch;
import frc.team6014.lib.math.Gearbox;
import frc.team6014.lib.util.Util;

public class ElevatorSubsystem extends SubsystemBase {

  public enum ElevatorControlState{
    HOMING,
    OPEN_LOOP,
    MOTION_MAGIC,
    TORQUE_CONTROL
  }

  private final TalonFX elevatorMaster = new TalonFX(30, Constants.CANIVORE_CANBUS);
  private final TalonFX elevatorSlave = new TalonFX(31, Constants.CANIVORE_CANBUS);

  private final MotionMagicTorqueCurrentFOC m_motionMagic = new MotionMagicTorqueCurrentFOC(0,0,0, false);
  private final PositionTorqueCurrentFOC m_torqueControl = new PositionTorqueCurrentFOC(0,0,1,false); 
  private final DutyCycleOut m_percentOut = new DutyCycleOut(0, true, false);

  private final Gearbox falconGearbox = new Gearbox(8, 60);
  private final double sprocketCircumferenceInCM = Units.inchesToMeters(1.790) * Math.PI * 100;
  private double targetOutput = 0.0;
  private SuperStructureState targetState = new SuperStructureState();
  private double lastDemandedHeight;

  private final LimitSwitch m_limitSwitch = new LimitSwitch(9);
  private final Debouncer m_switchBouncer = new Debouncer(0.2, DebounceType.kRising);
  
  public ElevatorControlState m_controlState = ElevatorControlState.OPEN_LOOP;

  private static ElevatorSubsystem mInstance;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    elevatorMaster.getConfigurator().apply(new TalonFXConfiguration());
    elevatorSlave.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 37.5;
    configs.Slot0.kI = 4.2;
    configs.Slot0.kD = 0.17;
    configs.Slot0.kS = 0.75;
    configs.Slot0.kV = 0.08;

    configs.Slot1.kP = 13;
    configs.Slot1.kI = 0.5;
    configs.Slot1.kD = 0.1;
    configs.Slot1.kS = 0.005;
    configs.Slot1.kV = 0;

    configs.Slot2.kP = 9.7;
    configs.Slot2.kI = 0.7;
    configs.Slot2.kD = 0.35;
    configs.Slot2.kS = 0.25;
    configs.Slot2.kV = 0.005;

    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -6;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 200;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 200;
    configs.MotionMagic.MotionMagicAcceleration = 425; // değiştir
    configs.MotionMagic.MotionMagicCruiseVelocity = 115; // değiştir
    configs.MotionMagic.MotionMagicJerk = 1050; //  değiştir

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
    configs.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    configs.CurrentLimits.StatorCurrentLimit = 500;
    configs.CurrentLimits.StatorCurrentLimitEnable = false;
    configs.CurrentLimits.SupplyCurrentLimit = 300;
    configs.CurrentLimits.SupplyCurrentLimitEnable = false;

    elevatorMaster.getConfigurator().apply(configs);
    elevatorSlave.getConfigurator().apply(configs);

    lastDemandedHeight = getHeight();
  }

  public static ElevatorSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new ElevatorSubsystem();
    }
    return mInstance;
  }

  @Override
  public void periodic() {
    //maybeShouldStop();

    switch(m_controlState){
      case OPEN_LOOP:
        setMotorOutput();
        break;
      case MOTION_MAGIC:
        setHeight();
        break;
      case TORQUE_CONTROL:
        holdPosition();
        break;
      default:
        stop();
        break;
    }

    switchTriggered();

    SmartDashboard.putString("Elevator State: ", m_controlState.toString());
    SmartDashboard.putNumber("Elevator Height", getHeight());
    SmartDashboard.putNumber("Elevator Current", getCurrent());
    SmartDashboard.putNumber("LastDemandElevator", lastDemandedHeight);
    //System.out.println(targetOutput);

    RobotState.getInstance().updateHeight(getHeight());
    // This method will be called once per scheduler run
  }

  public synchronized void setElevatorOpenLoop(double output){
    if(m_controlState != ElevatorControlState.OPEN_LOOP){
      m_controlState =  ElevatorControlState.OPEN_LOOP;
    }
    targetOutput = output;
    lastDemandedHeight = getHeight();
  }

  public synchronized void setElevatorPosition(SuperStructureState state){
    if(m_controlState != ElevatorControlState.MOTION_MAGIC){
      m_controlState =  ElevatorControlState.MOTION_MAGIC;
    }
    targetState = state;
    lastDemandedHeight = getHeight();
  }

  public synchronized void holdElevatorPosition(){
    if(m_controlState != ElevatorControlState.TORQUE_CONTROL){
      m_controlState =  ElevatorControlState.TORQUE_CONTROL;
    }
  }

  public void updateLastDemandedHeight(double height){
    lastDemandedHeight = height;
  }

  public void setElevatorControlState(ElevatorControlState state){
    m_controlState = state;
  }

  public void setMotorOutput(){
    elevatorMaster.setControl(m_percentOut.withOutput(targetOutput));
    elevatorSlave.setControl(m_percentOut.withOutput(targetOutput));
  }

  public void setHeight(){
    double sprocketRotation = targetState.getHeight() / sprocketCircumferenceInCM;
    if(targetState.getHeight() >= getHeight()){
    elevatorMaster.setControl(m_motionMagic.withPosition(sprocketRotation * falconGearbox.getRatio()).withSlot(0));
    elevatorSlave.setControl(m_motionMagic.withPosition(sprocketRotation * falconGearbox.getRatio()).withSlot(0));
    }else{
    elevatorMaster.setControl(m_motionMagic.withPosition(sprocketRotation * falconGearbox.getRatio()).withSlot(2));
    elevatorSlave.setControl(m_motionMagic.withPosition(sprocketRotation * falconGearbox.getRatio()).withSlot(2));
    }
  }

  public void holdPosition(){
    double sprocketRotation = lastDemandedHeight / sprocketCircumferenceInCM;
    elevatorMaster.setControl(m_torqueControl.withPosition(sprocketRotation * falconGearbox.getRatio()));
    elevatorSlave.setControl(m_torqueControl.withPosition(sprocketRotation * falconGearbox.getRatio()));
  }

  public void stop(){
    if(m_controlState != ElevatorControlState.HOMING){
      m_controlState =  ElevatorControlState.HOMING;
    }
    elevatorMaster.stopMotor();
    elevatorSlave.stopMotor();
  }

  public void overrideHeight(double height){
    double sprocketRotation = (height  / sprocketCircumferenceInCM);
    elevatorMaster.setRotorPosition(sprocketRotation * falconGearbox.getRatio());
    elevatorSlave.setRotorPosition(sprocketRotation * falconGearbox.getRatio());
  }

  public void resetToZero(){
    overrideHeight(39.9); //Min Height
  }

  public void resetToMax(){
    overrideHeight(125.047); //Max Height
  }

  public void switchTriggered(){
    if(m_switchBouncer.calculate(m_limitSwitch.isThereAThing()))
    overrideHeight(122.935);
  }

  public double getHeight(){
    var masterRot = elevatorMaster.getRotorPosition();
    var slaveRot = elevatorSlave.getRotorPosition();
    masterRot.refresh(); 
    slaveRot.refresh();
    double sprocketRotation = (masterRot.getValue() + slaveRot.getValue()) / 2 / falconGearbox.getRatio();
    return sprocketRotation * sprocketCircumferenceInCM;
  }

  public double getCurrent(){
    var masterCurrent = elevatorMaster.getStatorCurrent();
    var slaveCurrent = elevatorSlave.getStatorCurrent();
    masterCurrent.refresh();
    slaveCurrent.refresh();
    return (masterCurrent.getValue() + slaveCurrent.getValue()) / 2;
  }

  public void maybeShouldStop(){
    var currentVel = elevatorMaster.getRotorVelocity();
    currentVel.refresh();
    if(Util.epsilonEquals(currentVel.getValue() / falconGearbox.getRatio(), 0 , 0.1) && getCurrent() >= 100){//Kalibre ET!!!
      holdPosition();
      System.out.println("Triggered Elevator");
    }
  }

  public boolean isAtSetpoint(){
    return Math.abs(targetState.getHeight() - getHeight()) < 1; 
  } 

}
