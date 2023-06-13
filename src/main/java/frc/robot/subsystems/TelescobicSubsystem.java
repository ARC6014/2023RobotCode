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

  private static TelescobicSubsystem mInstance;
  /** Creates a new TelescobicArmSubsystem. */
  public TelescobicSubsystem() {

  }

  public static TelescobicSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new TelescobicSubsystem();
    }
    return mInstance;
  }

  @Override
  public void periodic() {
  
  }

  public synchronized void setTelescopicOpenLoop(double output){

  }

  public synchronized void setTelescopicPosition(SuperStructureState state){

  }

  public synchronized void holdTelescopicPosition(){

  }

  public void setTelescopicControlState(TelescopicControlState state){
  }

  public void updateLastDemandedLength(double length){
  }

  public void setMotorOutput(){

  }

  public void setLength(){

  }

  public void holdPosition(){

  }

  public void stop(){
  }

  public void overrideLength(double length){
  }

  public void resetToZero(){
  }

  public void resetToMax(){
  }

  public double getLength(){
    return 0;
  }

  public double getCurrent(){
    return 0;
  }

  public void maybeShouldStop(){

  }

  public boolean isAtSetpoint(){
    return true;
  } 


}
