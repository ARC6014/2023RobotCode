// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
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
  private final double sprocketCircumferenceInCM = Units.inchesToMeters(1.790) * Math.PI * 100;
  private final MotionMagicTorqueCurrentFOC m_motionMagic = new MotionMagicTorqueCurrentFOC(0,0,0, false);
  private double lastElevatorCommandHeight;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    elevatorMaster.getConfigurator().apply(new TalonFXConfiguration());
    elevatorSlave.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 20;
    configs.Slot0.kI = 5;
    configs.Slot0.kD = 0.5;
    configs.Slot0.kS = 5;
    configs.Slot0.kV = 0;

    configs.Voltage.PeakForwardVoltage = 10;
    configs.Voltage.PeakReverseVoltage = -10;
    configs.MotionMagic.MotionMagicAcceleration = 500; // değiştir
    configs.MotionMagic.MotionMagicCruiseVelocity = 400; // değiştir
    configs.MotionMagic.MotionMagicJerk = 300; //  değiştir

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
    configs.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    configs.CurrentLimits.StatorCurrentLimit = 500;
    configs.CurrentLimits.StatorCurrentLimitEnable = false;
    configs.CurrentLimits.SupplyCurrentLimit = 300;
    configs.CurrentLimits.SupplyCurrentLimitEnable = false;

    elevatorMaster.getConfigurator().apply(configs);
    elevatorSlave.getConfigurator().apply(configs);

    lastElevatorCommandHeight = getHeight();
  }

  @Override
  public void periodic() {
    //maybeShouldStop();
    SmartDashboard.putNumber("Elevator Height: ", getHeight());
    SmartDashboard.putNumber("Elevator Current: ", getCurrent());

    RobotState.getInstance().updateHeight(getHeight());
    // This method will be called once per scheduler run
  }

  public void setMotorOutput(double output){
    elevatorMaster.set(output);
    elevatorSlave.set(output);
    lastElevatorCommandHeight = getHeight();
    System.out.println(output);
  }

  public void setHeight(SuperStructureState state){
    double sprocketRotation = state.getHeight() / sprocketCircumferenceInCM;
    elevatorMaster.setControl(m_motionMagic.withPosition(sprocketRotation * falconGearbox.getRatio()));
    elevatorSlave.setControl(m_motionMagic.withPosition(sprocketRotation * falconGearbox.getRatio()));
    lastElevatorCommandHeight = getHeight();
  }

  public void holdCurrentPosition(){
    elevatorMaster.setControl(m_motionMagic.withPosition(lastElevatorCommandHeight));
    elevatorSlave.setControl(m_motionMagic.withPosition(lastElevatorCommandHeight));
  }

  public void stop(){
    elevatorMaster.set(0.0);
    elevatorSlave.set(0.0);
  }

  public void overrideHeight(double height){
    double sprocketRotation = (height  / sprocketCircumferenceInCM);
    elevatorMaster.setRotorPosition(sprocketRotation * falconGearbox.getRatio());
    elevatorSlave.setRotorPosition(sprocketRotation * falconGearbox.getRatio());
  }

  public void resetToZero(){
    overrideHeight(39.142); //Min Height
  }

  public void resetToMax(){
    overrideHeight(125.047); //Max Height
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
      elevatorMaster.setControl(m_motionMagic.withPosition(lastElevatorCommandHeight - 0.005));
      elevatorSlave.setControl(m_motionMagic.withPosition(lastElevatorCommandHeight - 0.005));
    }
  }

  public boolean isAtSetpoint(SuperStructureState state){
    return Math.abs(state.getHeight() - getHeight()) < 1; 
  } 

}
