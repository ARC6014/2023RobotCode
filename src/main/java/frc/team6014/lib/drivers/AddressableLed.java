// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014.lib.drivers;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AddressableLedConstants;
import io.github.oblarg.oblog.Loggable;

public class AddressableLed extends SubsystemBase implements Loggable {
  /** Creates a new AddressableLed. */
  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;

  private static AddressableLed m_instance;

  public AddressableLed() {

    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(AddressableLedConstants.ledPort);
    m_ledBuffer = new AddressableLEDBuffer(AddressableLedConstants.ledLength);

    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    // m_led.setData(m_ledBuffer);
    //setLEDColor(0, 255, 0);
    m_led.start();
  }

  public static AddressableLed getInstance() {
    if (m_instance == null) {
      m_instance = new AddressableLed();
    }
    return m_instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static void setLEDColor(Color color) {

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }
    m_led.setData(m_ledBuffer);
  }

  public void turnOffLED() {
    setLEDColor(Color.kBlack);
    m_led.setData(m_ledBuffer);
  }

}
