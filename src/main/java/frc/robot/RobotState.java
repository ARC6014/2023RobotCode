package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team6014.SuperStructureState;
import frc.team6014.lib.math.AllianceFlipUtil;

public class RobotState extends SubsystemBase{

    private static RobotState mInstance;

    public static RobotState getInstance(){
        if(mInstance == null){
            mInstance = new RobotState();
        }
        
        return mInstance;
    }

    public void periodic() {
        SmartDashboard.putString("State", m_targetPiece.toString());
        SmartDashboard.putString("Target Pose", m_targetPose.toString());
        SmartDashboard.putString("Level", m_level.toString());
        SmartDashboard.putString("Intake Level", m_intakeLevel.toString());
        SmartDashboard.putNumber("Absolute Height", currentStructureState.getAbsoluteHeight());
    }

    private SuperStructureState currentStructureState = new SuperStructureState();
    private pieceState m_targetPiece = pieceState.CONE;
    private scoreLevel m_level = scoreLevel.HOMING;
    private targetScorePose m_targetPose = targetScorePose.secondNode;
    private intakeLevel m_intakeLevel = intakeLevel.doubleStation;

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

    public enum intakeLevel {
        ground,
        doubleStation,
        singleStation
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

    public void setIntakeLevel(intakeLevel level){
        if(m_intakeLevel != level){
            m_intakeLevel = level;
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
                return 26;
            }
            return 29;
        }
        return 28.3;
    }

    public SuperStructureState getCurrentSuperStructureState(){
        return currentStructureState;
    }

    public scoreLevel getScoreTarget(){
        return m_level;
    }

    public SuperStructureState getTargetState(){
         if(m_level == scoreLevel.Intake){
            if(m_intakeLevel == intakeLevel.ground){
                if(m_targetPiece == pieceState.CUBE){
                    return intakeCubeFromGround;
                }
                return intakeConeFromGround;
            }
            if(m_intakeLevel == intakeLevel.doubleStation) return intakeFromDoubleStataion;
            if(m_intakeLevel == intakeLevel.singleStation) return intakeFromSingleStataion;

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
        return DriverStation.getAlliance() == Alliance.Blue ?
        new Pose2d(getScorePose().getX() + 0.3, getScorePose().getY(), getScorePose().getRotation()) :
        new Pose2d(getScorePose().getX() - 0.3, getScorePose().getY(), getScorePose().getRotation());
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

    private final SuperStructureState homingState = new SuperStructureState(125.4, 94.35, 5);
    private final SuperStructureState intakeFromDoubleStataion = new SuperStructureState(111.2, 94.4, 88.8);
    private final SuperStructureState intakeFromSingleStataion = new SuperStructureState(110, 94.4, 90);

    private final SuperStructureState intakeCubeFromGround = new SuperStructureState(100,94.4, -48.3);
    private final SuperStructureState groundCube = new SuperStructureState(120,94.4, 50);
    private final SuperStructureState firstCube = new SuperStructureState(115,94.75, 72.5);
    private final SuperStructureState secondCube = new SuperStructureState(124,105, 90);

    private final SuperStructureState intakeConeFromGround = new SuperStructureState(100,92.5, 90);
    private final SuperStructureState groundCone = new SuperStructureState(120,95, 50);
    private final SuperStructureState firstCone = new SuperStructureState(71,94.4, 123);
    private final SuperStructureState secondCone = new SuperStructureState(94.5,130, 118);

    private static final Pose2d firstNodePose = new Pose2d(1.965, 4.95, Rotation2d.fromDegrees(0));
    private static final Pose2d firstCubePose = new Pose2d(1.99, 4.40, Rotation2d.fromDegrees(0));
    private static final Pose2d secondNodePose = new Pose2d(1.965, 3.85, Rotation2d.fromDegrees(0));
    private static final Pose2d thirdNodePose = new Pose2d(1.965, 3.32, Rotation2d.fromDegrees(0));
    private static final Pose2d secondCubePose = new Pose2d(1.99, 2.75, Rotation2d.fromDegrees(0));
    private static final Pose2d fourthNodePose = new Pose2d(1.965, 2.18, Rotation2d.fromDegrees(0));
    private static final Pose2d fifthNodePose = new Pose2d(1.965, 1.62, Rotation2d.fromDegrees(0));
    private static final Pose2d thirdCubePose = new Pose2d(1.99, 1.065, Rotation2d.fromDegrees(0));
    private static final Pose2d sixthNodePose = new Pose2d(1.965, 0.50, Rotation2d.fromDegrees(0));
    private static final Pose2d loadingZonePose = new Pose2d(13.45, 7.00, Rotation2d.fromDegrees(0));
}
