package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team6014.SuperStructureState;
import frc.team6014.lib.math.AllianceFlipUtil;

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
    private targetScorePose m_targetPose = targetScorePose.secondNode;

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

    public enum targetScorePose{
        firstNode,
        firstCube,
        secondNode,
        thirdNode,
        secondCube,
        fourthNode,
        fifthNode,
        thirdCube,
        sixthNode
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

    public void setTargetPose(targetScorePose pose){
        if(m_targetPose != pose){
            m_targetPose = pose;
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
                return 24.5;
            }
            return 30.5;
        }
        return 30.05;
    }

    public SuperStructureState getCurrentSuperStructureState(){
        return currentStructureState;
    }

    public scoreLevel getScoreTarget(){
        return m_level;
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

    public Pose2d getTargetPose(){
        return new Pose2d(getScorePose().getX() + 0.2, getScorePose().getY(), getScorePose().getRotation());
    }

    public Pose2d getScorePose(){
        if(m_targetPose == targetScorePose.firstCube) return AllianceFlipUtil.apply(firstCubePose); 
        if(m_targetPose == targetScorePose.secondNode) return AllianceFlipUtil.apply(secondNodePose); 
        if(m_targetPose == targetScorePose.thirdNode) return AllianceFlipUtil.apply(thirdNodePose); 
        if(m_targetPose == targetScorePose.secondCube) return AllianceFlipUtil.apply(secondCubePose); 
        if(m_targetPose == targetScorePose.fourthNode) return AllianceFlipUtil.apply(fourthNodePose); 
        if(m_targetPose == targetScorePose.fifthNode) return AllianceFlipUtil.apply(fifthNodePose); 
        if(m_targetPose == targetScorePose.thirdCube) return AllianceFlipUtil.apply(thirdCubePose); 
        if(m_targetPose == targetScorePose.sixthNode) return AllianceFlipUtil.apply(sixthNodePose); 
        return AllianceFlipUtil.apply(firstNodePose);
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

    private static final Pose2d firstNodePose = new Pose2d(1.85, 4.95, Rotation2d.fromDegrees(0));
    private static final Pose2d firstCubePose = new Pose2d(1.85, 4.40, Rotation2d.fromDegrees(0));
    private static final Pose2d secondNodePose = new Pose2d(1.85, 3.85, Rotation2d.fromDegrees(0));
    private static final Pose2d thirdNodePose = new Pose2d(1.85, 3.30, Rotation2d.fromDegrees(0));
    private static final Pose2d secondCubePose = new Pose2d(1.85, 2.75, Rotation2d.fromDegrees(0));
    private static final Pose2d fourthNodePose = new Pose2d(1.85, 2.18, Rotation2d.fromDegrees(0));
    private static final Pose2d fifthNodePose = new Pose2d(1.85, 1.62, Rotation2d.fromDegrees(0));
    private static final Pose2d thirdCubePose = new Pose2d(1.85, 1.065, Rotation2d.fromDegrees(0));
    private static final Pose2d sixthNodePose = new Pose2d(1.85, 0.50, Rotation2d.fromDegrees(0));
    private static final Pose2d loadingZonePose = new Pose2d(13.45, 7.00, Rotation2d.fromDegrees(0));
}
