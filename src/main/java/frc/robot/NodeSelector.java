package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState.*;
import frc.team6014.lib.drivers.AddressableLed;

public class NodeSelector {

        private static NodeSelector mInstance;
        RobotState m_robotState = RobotState.getInstance();

        public static NodeSelector getInstance() {
                if (mInstance == null) {
                        mInstance = new NodeSelector();
                }

                return mInstance;
        }

        private static AddressableLed m_led = AddressableLed.getInstance();

        public void ConfigureWidgets() {

                // Ground Hybrid
                Shuffleboard.getTab("GamePieces")
                                .add("G9", new InstantCommand(() -> applyNode(0), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(0, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 0));
                
                Shuffleboard.getTab("GamePieces")
                                .add("G8", new InstantCommand(() -> applyNode(1), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(1, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 1));
                Shuffleboard.getTab("GamePieces")
                                .add("G7", new InstantCommand(() -> applyNode(2), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(2, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 2));
                Shuffleboard.getTab("GamePieces")
                                .add("G6", new InstantCommand(() -> applyNode(3), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(3, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 3));
                Shuffleboard.getTab("GamePieces")
                                .add("G5", new InstantCommand(() -> applyNode(4), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(4, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 4));
                Shuffleboard.getTab("GamePieces")
                                .add("G4", new InstantCommand(() -> applyNode(5), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(5, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 5));
                Shuffleboard.getTab("GamePieces")
                                .add("G3", new InstantCommand(() -> applyNode(6), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(6, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 6));
                Shuffleboard.getTab("GamePieces")
                                .add("G2", new InstantCommand(() -> applyNode(7), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(7, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 7));
                Shuffleboard.getTab("GamePieces")
                                .add("G1", new InstantCommand(() -> applyNode(8), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(8, 0)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 8));

                // First Level - Cube
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cube3", new InstantCommand(() -> applyNode(10), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(1, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 10));
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cube2", new InstantCommand(() -> applyNode(13), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(4, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 13));
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cube1", new InstantCommand(() -> applyNode(16), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(7, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 16));

                // First Level - Cone
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cone6", new InstantCommand(() -> applyNode(9), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(0, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 9));
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cone5", new InstantCommand(() -> applyNode(11), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(2, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 11));
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cone4", new InstantCommand(() -> applyNode(12), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand).withWidget(BuiltInWidgets.kCommand)
                                .withPosition(3, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 12));
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cone3", new InstantCommand(() -> applyNode(14), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand).withWidget(BuiltInWidgets.kCommand)
                                .withPosition(5, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 14));
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cone2", new InstantCommand(() -> applyNode(15), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand).withWidget(BuiltInWidgets.kCommand)
                                .withPosition(6, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 15));
                Shuffleboard.getTab("GamePieces")
                                .add("1ST-Cone1", new InstantCommand(() -> applyNode(17), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand).withWidget(BuiltInWidgets.kCommand)
                                .withPosition(8, 1)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 17));

                // Second Level - Cube
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cube3", new InstantCommand(() -> applyNode(19), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(1, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 19));
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cube2", new InstantCommand(() -> applyNode(22), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(4, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 22));
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cube1", new InstantCommand(() -> applyNode(25), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(7, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 25));

                // Second Level - Cone
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cone6", new InstantCommand(() -> applyNode(18), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(0, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 18));
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cone5", new InstantCommand(() -> applyNode(20), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(2, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 20));
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cone4", new InstantCommand(() -> applyNode(21), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(3, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 21));
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cone3", new InstantCommand(() -> applyNode(23), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(5, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 23));
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cone2", new InstantCommand(() -> applyNode(24), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(6, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 24));
                Shuffleboard.getTab("GamePieces")
                                .add("2ND-Cone1", new InstantCommand(() -> applyNode(26), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(8, 2)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 26));
                Shuffleboard.getTab("GamePieces")
                                .add("Ground Intake", new InstantCommand(() -> applyNode(50), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(3, 4)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 50));
                Shuffleboard.getTab("GamePieces")
                                .add("Double Intake", new InstantCommand(() -> applyNode(51), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(4, 4)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 51));
                Shuffleboard.getTab("GamePieces")
                                .add("Single Intake", new InstantCommand(() -> applyNode(52), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(5, 4)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 52));
                Shuffleboard.getTab("GamePieces")
                                .add("Cone", new InstantCommand(() -> applyNode(53), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(4, 3)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 53));
                Shuffleboard.getTab("GamePieces")
                                .add("Cube", new InstantCommand(() -> applyNode(54), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(5, 3)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 54));
                Shuffleboard.getTab("GamePieces")
                                .add("Homing", new InstantCommand(() -> applyNode(55), m_robotState))
                                .withWidget(BuiltInWidgets.kCommand)
                                .withPosition(3, 3)
                                .withSize(1, 1)
                                .withProperties(Map.of("ID", 55));
        }

        public void applyNode(int id) {
                id = transformID(id);

                // For Ground (Hybrid) Positions
                if (id == 0) {
                        //RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        m_robotState.setScoreLevel(scoreLevel.Ground);
                        m_robotState.setTargetPose(targetScorePose.sixthNode);
                        m_led.nodeTriggered(true);
                        System.out.println("G added");
                }
                if (id == 1) {
                        //RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        m_robotState.setScoreLevel(scoreLevel.Ground);
                        m_robotState.setTargetPose(targetScorePose.thirdCube);
                        m_led.nodeTriggered(true);
                        System.out.println("G added");
                }
                if (id == 2) {
                        //RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        m_robotState.setScoreLevel(scoreLevel.Ground);
                        m_robotState.setTargetPose(targetScorePose.fifthNode);
                        m_led.nodeTriggered(true);
                        System.out.println("G added");
                }
                if (id == 3) {
                        //RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        m_robotState.setScoreLevel(scoreLevel.Ground);
                        m_robotState.setTargetPose(targetScorePose.fourthNode);
                        m_led.nodeTriggered(true);
                        System.out.println("G added");
                }
                if (id == 4) {
                        //RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        m_robotState.setScoreLevel(scoreLevel.Ground);
                        m_robotState.setTargetPose(targetScorePose.secondCube);
                        m_led.nodeTriggered(true);
                        System.out.println("G added");
                }
                if (id == 5) {
                        //RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        m_robotState.setScoreLevel(scoreLevel.Ground);
                        m_robotState.setTargetPose(targetScorePose.thirdNode);
                        m_led.nodeTriggered(true);
                        System.out.println("G added");
                }
                if (id == 6) {
                        //RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        m_robotState.setScoreLevel(scoreLevel.Ground);
                        m_robotState.setTargetPose(targetScorePose.secondNode);
                        m_led.nodeTriggered(true);
                        System.out.println("G added");
                }
                if (id == 7) {
                        //RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        m_robotState.setScoreLevel(scoreLevel.Ground);
                        m_robotState.setTargetPose(targetScorePose.firstCube);
                        m_led.nodeTriggered(true);
                        System.out.println("G added");
                }
                if (id == 8) {
                        //RobotState.getInstance().setCone(); // TODO: Change this for ground mode
                        m_robotState.setScoreLevel(scoreLevel.Ground);
                        m_robotState.setTargetPose(targetScorePose.firstNode);
                        m_led.nodeTriggered(true);
                        System.out.println("G added");
                }

                // For First Level Placement
                if (id == 9) {
                        m_robotState.setCone();
                        m_robotState.setScoreLevel(scoreLevel.FirstLevel);
                        m_robotState.setTargetPose(targetScorePose.sixthNode);
                        m_led.nodeTriggered(true);
                        System.out.println("FL added");
                }
                if (id == 10) {
                        m_robotState.setCube();
                        m_robotState.setScoreLevel(scoreLevel.FirstLevel);
                        m_robotState.setTargetPose(targetScorePose.thirdCube);
                        m_led.nodeTriggered(true);
                        System.out.println("FL added");
                }
                if (id == 11) {
                        m_robotState.setCone();
                        m_robotState.setScoreLevel(scoreLevel.FirstLevel);
                        m_robotState.setTargetPose(targetScorePose.fifthNode);
                        m_led.nodeTriggered(true);
                        System.out.println("FL added");
                }
                if (id == 12) {
                        m_robotState.setCone();
                        m_robotState.setScoreLevel(scoreLevel.FirstLevel);
                        m_robotState.setTargetPose(targetScorePose.fourthNode);
                        m_led.nodeTriggered(true);
                        System.out.println("FL added");
                }
                if (id == 13) {
                        m_robotState.setCube();
                        m_robotState.setScoreLevel(scoreLevel.FirstLevel);
                        m_robotState.setTargetPose(targetScorePose.secondCube);
                        m_led.nodeTriggered(true);
                        System.out.println("FL added");
                }
                if (id == 14) {
                        m_robotState.setCone();
                        m_robotState.setScoreLevel(scoreLevel.FirstLevel);
                        m_robotState.setTargetPose(targetScorePose.thirdNode);
                        m_led.nodeTriggered(true);
                        System.out.println("FL added");
                }
                if (id == 15) {
                        m_robotState.setCone();
                        m_robotState.setScoreLevel(scoreLevel.FirstLevel);
                        m_robotState.setTargetPose(targetScorePose.secondNode);
                        m_led.nodeTriggered(true);
                        System.out.println("FL added");
                }
                if (id == 16) {
                        m_robotState.setCube();
                        m_robotState.setScoreLevel(scoreLevel.FirstLevel);
                        m_robotState.setTargetPose(targetScorePose.firstCube);
                        m_led.nodeTriggered(true);
                        System.out.println("FL added");
                }
                if (id == 17) {
                        m_robotState.setCone();
                        m_robotState.setScoreLevel(scoreLevel.FirstLevel);
                        m_robotState.setTargetPose(targetScorePose.firstNode);
                        m_led.nodeTriggered(true);
                        System.out.println("FL added");
                }

                // For Second-Level Placement
                if (id == 18) {
                        m_robotState.setCone();
                        m_robotState.setScoreLevel(scoreLevel.SecondLevel);
                        m_robotState.setTargetPose(targetScorePose.sixthNode);
                        m_led.nodeTriggered(true);
                        System.out.println("SL added");
                }
                if (id == 19) {
                        m_robotState.setCube();
                        m_robotState.setScoreLevel(scoreLevel.SecondLevel);
                        m_robotState.setTargetPose(targetScorePose.thirdCube);
                        m_led.nodeTriggered(true);
                        System.out.println("SL added");
                }
                if (id == 20) {
                        m_robotState.setCone();
                        m_robotState.setScoreLevel(scoreLevel.SecondLevel);
                        m_robotState.setTargetPose(targetScorePose.fifthNode);
                        m_led.nodeTriggered(true);
                        System.out.println("SL added");
                }
                if (id == 21) {
                        m_robotState.setCone();
                        m_robotState.setScoreLevel(scoreLevel.SecondLevel);
                        m_robotState.setTargetPose(targetScorePose.fourthNode);
                        m_led.nodeTriggered(true);
                        System.out.println("SL added");
                }
                if (id == 22) {
                        m_robotState.setCube();
                        m_robotState.setScoreLevel(scoreLevel.SecondLevel);
                        m_robotState.setTargetPose(targetScorePose.secondCube);
                        m_led.nodeTriggered(true);
                        System.out.println("SL added");
                }
                if (id == 23) {
                        m_robotState.setCone();
                        m_robotState.setScoreLevel(scoreLevel.SecondLevel);
                        m_robotState.setTargetPose(targetScorePose.thirdNode);
                        m_led.nodeTriggered(true);
                        System.out.println("SL added");
                }
                if (id == 24) {
                        m_robotState.setCone();
                        m_robotState.setScoreLevel(scoreLevel.SecondLevel);
                        m_robotState.setTargetPose(targetScorePose.secondNode);
                        m_led.nodeTriggered(true);
                        System.out.println("SL added");
                }
                if (id == 25) {
                        m_robotState.setCube();
                        m_robotState.setScoreLevel(scoreLevel.SecondLevel);
                        m_robotState.setTargetPose(targetScorePose.firstCube);
                        m_led.nodeTriggered(true);
                        System.out.println("SL added");
                }
                if (id == 26) {
                        m_robotState.setCone();
                        m_robotState.setScoreLevel(scoreLevel.SecondLevel);
                        m_robotState.setTargetPose(targetScorePose.firstNode);
                        m_led.nodeTriggered(true);
                        System.out.println("SL added");
                }
                
                if (id == 50) {
                        m_robotState.setScoreLevel(scoreLevel.Intake);
                        m_robotState.setIntakeLevel(intakeLevel.ground);
                        
                }
                if (id == 51) {
                        m_robotState.setScoreLevel(scoreLevel.Intake);
                        m_robotState.setIntakeLevel(intakeLevel.doubleStation);
                        
                }
                if (id == 52) {
                        m_robotState.setScoreLevel(scoreLevel.Intake);
                        m_robotState.setIntakeLevel(intakeLevel.singleStation);
                        
                }
                if (id == 53) {
                        m_robotState.setCone();
                }
                if (id == 54) {
                        m_robotState.setCube();
                }
                if (id == 55) {
                        m_robotState.setScoreLevel(scoreLevel.HOMING);
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
