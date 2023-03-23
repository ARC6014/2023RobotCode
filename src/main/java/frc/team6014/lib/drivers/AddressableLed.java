// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014.lib.drivers;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.AddressableLedConstants;
import frc.robot.RobotState.pieceState;
import io.github.oblarg.oblog.Loggable;

public class AddressableLed extends SubsystemBase implements Loggable {
  /** Creates a new AddressableLed. */
  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;
  private boolean triggered = false;
  private final Timer m_timer = new Timer();

  private static AddressableLed m_instance;

  public AddressableLed(int iport) {

    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(iport);
    m_ledBuffer = new AddressableLEDBuffer(40);

    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    // m_led.setData(m_ledBuffer);
    m_led.start();
    /*for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, Color.kGreen);
    }
    m_led.setData(m_ledBuffer);*/
  }


  @Override
  public void periodic() {
    if(RobotState.getInstance().getPiece() == pieceState.CONE){
      setLEDColor(Color.kRed);
    }if(RobotState.getInstance().getPiece() == pieceState.CUBE){
      setLEDColor(Color.kGreen);      
    }
    setLEDTailColor(Color.kBlack);
    if(m_timer.get() > 0.8){
      m_timer.stop();
      setTriggered(false);
      m_timer.reset();
    }
    // This method will be called once per scheduler run
  }

  public void setLEDColor(Color color) {
    if(triggered){
      color = Color.kBlue;
    }
    for (int i = 0; i < m_ledBuffer.getLength()-5; i++) {
      m_ledBuffer.setLED(i, color);
    }
    m_led.setData(m_ledBuffer);

  }

    public void setLEDTailColor(Color color) {
        for (int i = m_ledBuffer.getLength(); i >= m_ledBuffer.getLength()-5; i--) {
        m_ledBuffer.setLED(i, color);
        }
        m_led.setData(m_ledBuffer);

    }

  public void setTriggered(boolean state){
    triggered = state;
    m_timer.start();
  }

  public boolean getTriggered(){
    return triggered;
  }

  public void turnOffLED() {
    setLEDColor(Color.kBlack);
    m_led.setData(m_ledBuffer);
  }

}
