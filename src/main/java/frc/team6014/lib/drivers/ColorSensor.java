// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.team6014.lib.drivers;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. 
public class ColorSensor{

    private final ColorSensorV3 m_colorSensor;
    private final IndexingSensor m_indexer;
    private final ColorMatch m_colorMatcher = new ColorMatch();

    //TO DO: Calibrate it!!!
    //red - green - blue (0-1)
    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);

    public ColorSensor(int indexerChannel, double kDistanceConstant){
        m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        m_indexer = new IndexingSensor(indexerChannel, kDistanceConstant);

        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kRedTarget);

    }

    public IndexingSensor getIndexingSensor(){
        return m_indexer;
    }

    public boolean isWrongBall(){
        boolean proximity = m_colorSensor.getProximity() >=90; //TO DO: CALIBRATE İT
        boolean blue = getColor() == kBlueTarget;
        boolean red = getColor() == kRedTarget;
        if (DriverStation.getAlliance().equals(Alliance.Red) && blue && proximity) {
            return true;
        } else if (DriverStation.getAlliance().equals(Alliance.Blue) && red && proximity) {
            return true;
        } else {
            return false;
        }
    }

    public Color getColor(){
        if(m_indexer.isThereAThing()){
            Color detectedColor = m_colorSensor.getColor();
            ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
            if(match.color == kBlueTarget){
                return kBlueTarget;
            }
            if(match.color == kRedTarget){
                return kRedTarget;
            }else{
                return null;
            }
        }else{
            return null;
        }
    }

}
*/