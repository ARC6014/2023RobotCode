package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState.*;

public class NodeSelector {

        private static NodeSelector mInstance;
        RobotState m_robotState = RobotState.getInstance();

        public static NodeSelector getInstance() {
                if (mInstance == null) {
                        mInstance = new NodeSelector();
                }

                return mInstance;
        }

        public void ConfigureWidgets() {

                // Ground Hybrid
                Shuffleboard.getTab("GamePieces")
                                .add("G9", new InstantCommand(() -> applyNode(0)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(8, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 0));
                Shuffleboard.getTab("GamePieces")
                                .add("G8", new InstantCommand(() -> applyNode(1)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(7, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 1));
                Shuffleboard.getTab("GamePieces")
                                .add("G7", new InstantCommand(() -> applyNode(2)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(6, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 2));
                Shuffleboard.getTab("GamePieces")
                                .add("G6", new InstantCommand(() -> applyNode(3)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(5, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 3));
                Shuffleboard.getTab("GamePieces")
                                .add("G5", new InstantCommand(() -> applyNode(4)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(4, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 4));
                Shuffleboard.getTab("GamePieces")
                                .add("G4", new InstantCommand(() -> applyNode(5)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(3, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 5));
                Shuffleboard.getTab("GamePieces")
                                .add("G3", new InstantCommand(() -> applyNode(6)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(2, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 6));
                Shuffleboard.getTab("GamePieces")
                                .add("G2", new InstantCommand(() -> applyNode(7)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(1, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 7));
                Shuffleboard.getTab("GamePieces")
                                .add("G1", new InstantCommand(() -> applyNode(8)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(0, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 8));

                // First Level - Cube
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cube3", new InstantCommand(() -> applyNode(10)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(1, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 10));
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cube2", new InstantCommand(() -> applyNode(13)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(4, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 13));
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cube1", new InstantCommand(() -> applyNode(16)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(7, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 16));

                // First Level - Cone
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cone6", new InstantCommand(() -> applyNode(9)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(0, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 9));
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cone5", new InstantCommand(() -> applyNode(11)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(2, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 11));
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cone4", new InstantCommand(() -> applyNode(12)))
                                .withWidget(BuiltInWidgets.kCommand).withWidget(BuiltInWidgets.kCommand)
                                .withPosition(3, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 12));
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cone3", new InstantCommand(() -> applyNode(14)))
                                .withWidget(BuiltInWidgets.kCommand).withWidget(BuiltInWidgets.kCommand)
                                .withPosition(5, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 14));
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cone2", new InstantCommand(() -> applyNode(15)))
                                .withWidget(BuiltInWidgets.kCommand).withWidget(BuiltInWidgets.kCommand)
                                .withPosition(6, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 15));
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cone1", new InstantCommand(() -> applyNode(17)))
                                .withWidget(BuiltInWidgets.kCommand).withWidget(BuiltInWidgets.kCommand)
                                .withPosition(8, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 17));

                // Second Level - Cube
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cube3", new InstantCommand(() -> applyNode(19)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(1, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 19));
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cube2", new InstantCommand(() -> applyNode(22)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(4, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 22));
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cube1", new InstantCommand(() -> applyNode(25)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(7, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 25));

                // Second Level - Cone
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cone6", new InstantCommand(() -> applyNode(18)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(0, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 18));
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cone5", new InstantCommand(() -> applyNode(20)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(2, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 20));
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cone4", new InstantCommand(() -> applyNode(21)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(3, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 21));
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cone3", new InstantCommand(() -> applyNode(23)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(5, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 23));
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cone2", new InstantCommand(() -> applyNode(24)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(6, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 24));
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cone1", new InstantCommand(() -> applyNode(26)))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(8, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 26));
        }

        public void applyNode(int id) {
                id = transformID(id);

                // For Ground (Hybrid) Positions
                if (id == 0) {
                        RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        RobotState.getInstance().setScoreLevel(scoreLevel.Ground);
                        RobotState.getInstance().setTargetPose(targetScorePose.sixthNode);
                }
                if (id == 1) {
                        RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        RobotState.getInstance().setScoreLevel(scoreLevel.Ground);
                        RobotState.getInstance().setTargetPose(targetScorePose.thirdCube);
                }
                if (id == 2) {
                        RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        RobotState.getInstance().setScoreLevel(scoreLevel.Ground);
                        RobotState.getInstance().setTargetPose(targetScorePose.fifthNode);
                }
                if (id == 3) {
                        RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        RobotState.getInstance().setScoreLevel(scoreLevel.Ground);
                        RobotState.getInstance().setTargetPose(targetScorePose.fourthNode);
                }
                if (id == 4) {
                        RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        RobotState.getInstance().setScoreLevel(scoreLevel.Ground);
                        RobotState.getInstance().setTargetPose(targetScorePose.secondCube);
                }
                if (id == 5) {
                        RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        RobotState.getInstance().setScoreLevel(scoreLevel.Ground);
                        RobotState.getInstance().setTargetPose(targetScorePose.thirdNode);
                }
                if (id == 6) {
                        RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        RobotState.getInstance().setScoreLevel(scoreLevel.Ground);
                        RobotState.getInstance().setTargetPose(targetScorePose.secondNode);
                }
                if (id == 7) {
                        RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        RobotState.getInstance().setScoreLevel(scoreLevel.Ground);
                        RobotState.getInstance().setTargetPose(targetScorePose.firstCube);
                }
                if (id == 8) {
                        RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        RobotState.getInstance().setScoreLevel(scoreLevel.Ground);
                        RobotState.getInstance().setTargetPose(targetScorePose.firstNode);
                }

                // For First Level Placement
                if (id == 9) {
                        RobotState.getInstance().setCone();
                        RobotState.getInstance().setScoreLevel(scoreLevel.FirstLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.sixthNode);
                }
                if (id == 10) {
                        RobotState.getInstance().setCube();
                        RobotState.getInstance().setScoreLevel(scoreLevel.FirstLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.thirdCube);
                }
                if (id == 11) {
                        RobotState.getInstance().setCone();
                        RobotState.getInstance().setScoreLevel(scoreLevel.FirstLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.fifthNode);
                }
                if (id == 12) {
                        RobotState.getInstance().setCone();
                        RobotState.getInstance().setScoreLevel(scoreLevel.FirstLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.fourthNode);
                }
                if (id == 13) {
                        RobotState.getInstance().setCube();
                        RobotState.getInstance().setScoreLevel(scoreLevel.FirstLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.secondCube);
                }
                if (id == 14) {
                        RobotState.getInstance().setCone();
                        RobotState.getInstance().setScoreLevel(scoreLevel.FirstLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.thirdNode);
                }
                if (id == 15) {
                        RobotState.getInstance().setCone();
                        RobotState.getInstance().setScoreLevel(scoreLevel.FirstLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.secondNode);
                }
                if (id == 16) {
                        RobotState.getInstance().setCube();
                        RobotState.getInstance().setScoreLevel(scoreLevel.FirstLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.firstCube);
                }
                if (id == 17) {
                        RobotState.getInstance().setCone();
                        RobotState.getInstance().setScoreLevel(scoreLevel.FirstLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.firstNode);
                }

                // For Second-Level Placement
                if (id == 18) {
                        RobotState.getInstance().setCone();
                        RobotState.getInstance().setScoreLevel(scoreLevel.SecondLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.sixthNode);
                }
                if (id == 19) {
                        RobotState.getInstance().setCube();
                        RobotState.getInstance().setScoreLevel(scoreLevel.SecondLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.thirdCube);
                }
                if (id == 20) {
                        RobotState.getInstance().setCone();
                        RobotState.getInstance().setScoreLevel(scoreLevel.SecondLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.fifthNode);
                }
                if (id == 21) {
                        RobotState.getInstance().setCone();
                        RobotState.getInstance().setScoreLevel(scoreLevel.SecondLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.fourthNode);
                }
                if (id == 22) {
                        RobotState.getInstance().setCube();
                        RobotState.getInstance().setScoreLevel(scoreLevel.SecondLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.secondCube);
                }
                if (id == 23) {
                        RobotState.getInstance().setCone();
                        RobotState.getInstance().setScoreLevel(scoreLevel.SecondLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.thirdNode);
                }
                if (id == 24) {
                        RobotState.getInstance().setCone();
                        RobotState.getInstance().setScoreLevel(scoreLevel.SecondLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.secondNode);
                }
                if (id == 25) {
                        RobotState.getInstance().setCube();
                        RobotState.getInstance().setScoreLevel(scoreLevel.SecondLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.firstCube);
                }
                if (id == 26) {
                        RobotState.getInstance().setCone();
                        RobotState.getInstance().setScoreLevel(scoreLevel.SecondLevel);
                        RobotState.getInstance().setTargetPose(targetScorePose.firstNode);
                }
        }

        public int transformID(int index) {

                if (DriverStation.getAlliance() == Alliance.Red) {
                        return index;
                }

                else {
                        if (index >= 0 && index <= 8) {
                                index = 8 - index;
                        }

                        else if (index >= 9 && index <= 17) {
                                index = 26 - index;
                        }

                        else if (index >= 18 && index <= 26) {
                                index = 44 - index;
                        }
                        
                        return index;
                }
                

        }

}
