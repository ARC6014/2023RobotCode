// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014.lib.drivers;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class LimitSwitch {
    private final DigitalInput m_limitSwitch;
    

    
    public LimitSwitch(int channel) {
        m_limitSwitch = new DigitalInput(channel);
    }

    public boolean isThereAThing() {
        return !m_limitSwitch.get(); 
    }


}
