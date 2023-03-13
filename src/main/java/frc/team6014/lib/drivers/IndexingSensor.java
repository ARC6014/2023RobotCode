// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014.lib.drivers;

import edu.wpi.first.wpilibj.AnalogInput;

/** Add your docs here. */
public class IndexingSensor {
    private final AnalogInput m_indexer;
    private final double m_distanceConstant;

    public IndexingSensor(int channel, double kDistanceConstant){
        m_indexer = new AnalogInput(channel);
        m_distanceConstant = kDistanceConstant;
    }

    public boolean isThereAThing(){
        if(getValue() >= m_distanceConstant){
            return true;
        }
        return false;
        
    }

    public double getValue(){
        return m_indexer.getVoltage();
    }
}
