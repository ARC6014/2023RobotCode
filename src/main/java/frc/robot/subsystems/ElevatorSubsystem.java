// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.ControlRequest;
import com.ctre.phoenixpro.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team6014.SuperStructureState;
import frc.team6014.lib.math.Gearbox;
import frc.team6014.lib.util.Util;

public class ElevatorSubsystem extends SubsystemBase {

  private static ElevatorSubsystem mInstance;

  public static synchronized ElevatorSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new ElevatorSubsystem();
    }
    return mInstance;
  }

  private final TalonFX elevatorMaster = new TalonFX(30, Constants.CANIVORE_CANBUS);
  private final TalonFX elevatorSlave = new TalonFX(31, Constants.CANIVORE_CANBUS);
  private final Gearbox falconGearbox = new Gearbox(8, 60);
  private final MotionMagicTorqueCurrentFOC m_motionMagic = new MotionMagicTorqueCurrentFOC(0,0,0, false);
  private double lastHeight;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    elevatorMaster.getConfigurator().apply(new TalonFXConfiguration());
    elevatorSlave.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 250;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 0;
    configs.Slot0.kS = 0;
    configs.Slot0.kV = 0;

   // configs.Voltage.PeakForwardVoltage = 10;
   // configs.Voltage.PeakReverseVoltage = -10;
    configs.MotionMagic.MotionMagicAcceleration = 200; // değiştir
    configs.MotionMagic.MotionMagicCruiseVelocity = 200; // değiştir
    configs.MotionMagic.MotionMagicJerk = 100; //  değiştir

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
    configs.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    configs.CurrentLimits.StatorCurrentLimit = 300;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 150;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    elevatorMaster.getConfigurator().apply(configs);
    elevatorSlave.getConfigurator().apply(configs);

    lastHeight = getHeight();
  }

  @Override
  public void periodic() {
    //maybeShouldStop();
    SmartDashboard.putNumber("Elevator Height", getHeight());
    System.out.println(getHeight());
    // This method will be called once per scheduler run
  }

  public synchronized void overrideHeight(double height){
    double sprocketRotation = (height / (Units.inchesToMeters(1.790) * Math.PI));
    elevatorMaster.setRotorPosition(sprocketRotation * falconGearbox.getRatio());
    elevatorSlave.setRotorPosition(sprocketRotation * falconGearbox.getRatio());
  }

  public synchronized double getHeight(){
    var masterRot = elevatorMaster.getRotorPosition();
    var slaveRot = elevatorSlave.getRotorPosition();
    masterRot.refresh(); 
    slaveRot.refresh();
    double sprocketRotation = (masterRot.getValue() + slaveRot.getValue()) / 2 / falconGearbox.getRatio();
    return sprocketRotation * Units.inchesToMeters(1.790) * Math.PI;
  }

  public synchronized void setHeight(SuperStructureState state){
    double sprocketRotation = (state.getHeight() / (Units.inchesToMeters(1.790) * Math.PI));
    elevatorMaster.setControl(m_motionMagic.withPosition(sprocketRotation * falconGearbox.getRatio()));
    elevatorSlave.setControl(m_motionMagic.withPosition(sprocketRotation * falconGearbox.getRatio()));
    lastHeight = getHeight();
  }

  public synchronized void setMotorOutput(double output){
    elevatorMaster.set(output);
    elevatorSlave.set(output);
    lastHeight = getHeight();
  }

  public synchronized double getCurrent(){
    var masterCurrent = elevatorMaster.getStatorCurrent();
    var slaveCurrent = elevatorSlave.getStatorCurrent();
    masterCurrent.refresh();
    slaveCurrent.refresh();
    return (masterCurrent.getValue() + slaveCurrent.getValue()) / 2;
  }

  public synchronized void holdCurrentPosition(){
    elevatorMaster.setControl(m_motionMagic.withPosition(lastHeight));
    elevatorSlave.setControl(m_motionMagic.withPosition(lastHeight));
  }

  public synchronized void stop(){
    elevatorMaster.set(0.0);
    elevatorSlave.set(0.0);
  }

  public synchronized boolean isSetPoint(SuperStructureState state){
    return Math.abs(state.getHeight() - getHeight()) < 0.01;
  } 

  public synchronized void maybeShouldStop(){
    var currentVel = elevatorMaster.getRotorVelocity();
    currentVel.refresh();
    if(Util.epsilonEquals(currentVel.getValue() / falconGearbox.getRatio(), 0 , 0.1) && getCurrent() >= 80){
      elevatorMaster.setControl(m_motionMagic.withPosition(lastHeight - 0.005));
      elevatorSlave.setControl(m_motionMagic.withPosition(lastHeight - 0.005));
    }
  }
}
