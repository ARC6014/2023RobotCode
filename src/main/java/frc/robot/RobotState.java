package frc.robot;

import frc.team6014.SuperStructureState;

public class RobotState{

    private static RobotState mInstance;

    public static RobotState getInstance(){
        if(mInstance == null){
            mInstance = new RobotState();
        }
        
        return mInstance;
    }

    private SuperStructureState currentStructureState = new SuperStructureState();
    private pieceState m_targetPiece = pieceState.CONE;
    private scoreLevel m_level = scoreLevel.HOMING;

    public enum pieceState {
        CUBE,
        CONE
    }

    public enum scoreLevel {
        Intake,
        Ground,
        FirstLevel,
        SecondLevel,
        HOMING
    }

    public void setCube() {
        if(m_targetPiece != pieceState.CUBE){
            m_targetPiece = pieceState.CUBE;
        }
    }

    public void setCone() {
        if(m_targetPiece != pieceState.CONE){
            m_targetPiece = pieceState.CONE;
        }
    }

    public pieceState getPiece(){
        return m_targetPiece;
    }

    public void setScoreLevel(scoreLevel level){
        if(m_level != level){
            m_level = level;
        }
    }

    public void updateHeight(double height){
        currentStructureState.setHeight(height);
    }

    public void updateLength(double length){
        currentStructureState.setLength(length);
    }

    public void updateDegree(double degree){
        currentStructureState.setDegree(degree);
    }

    public double getEndEffector(){
        if(m_targetPiece == pieceState.CUBE){
            if(currentStructureState.getDegree() < - 17.5){
                return 23;
            }
            return 30.5;
        }
        return 30.05;
    }

    public SuperStructureState getCurrentSuperStructureState(){
        return currentStructureState;
    }

    public SuperStructureState getTargetState(){
         if(m_level == scoreLevel.Intake){
            if(m_targetPiece == pieceState.CUBE){
                return intakeCube;
            }
            return intakeCone;
        }else if(m_level == scoreLevel.Ground){
            if(m_targetPiece == pieceState.CUBE){
                return groundCube;
            }
            return groundCone;
        }else if(m_level == scoreLevel.FirstLevel){
            if(m_targetPiece == pieceState.CUBE){
                return firstCube;
            }
            return firstCone;
        }else if(m_level == scoreLevel.SecondLevel){
            if(m_targetPiece == pieceState.CUBE){
                return secondCube;
            }
            return secondCone;
        }
        return homingState;
    }

    private final SuperStructureState homingState = new SuperStructureState(120, 92.2, -20);

    private final SuperStructureState intakeCube = new SuperStructureState(105,92.2, -40);
    private final SuperStructureState groundCube = new SuperStructureState(120,92.5, 50);
    private final SuperStructureState firstCube = new SuperStructureState(115,93, 72.5);
    private final SuperStructureState secondCube = new SuperStructureState(124,105, 90);

    private final SuperStructureState intakeCone = new SuperStructureState(100,92.5, 90);
    private final SuperStructureState groundCone = new SuperStructureState(120,95, 50);
    private final SuperStructureState firstCone = new SuperStructureState(71,93, 124);
    private final SuperStructureState secondCone = new SuperStructureState(96,130, 115);


}
