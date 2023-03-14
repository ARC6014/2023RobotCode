package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
                .add("Hybrid", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(0, 2)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 3));
        Shuffleboard.getTab("GamePieces")
                .add("Hybrid", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(1, 2)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 1));
        Shuffleboard.getTab("GamePieces")
                .add("Hybrid", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(2, 2)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 2));

        Shuffleboard.getTab("GamePieces")
                .add("Hybrid", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(3, 2)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 3));
        Shuffleboard.getTab("GamePieces")
                .add("Hybrid", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(4, 2)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 4));
        Shuffleboard.getTab("GamePieces")
                .add("Hybrid", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(5, 2)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 5));

        Shuffleboard.getTab("GamePieces")
                .add("Hybrid", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(6, 2)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 6));
        Shuffleboard.getTab("GamePieces")
                .add("Hybrid", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(7, 2)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 7));
        Shuffleboard.getTab("GamePieces")
                .add("Hybrid", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(8, 2)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 8));



        // First Level - Cube
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL1 Cube", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(1, 1)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 10));
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL1 Cube", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(4, 1)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 13));
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL1 Cube", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(7, 1)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 16));

        // First Level - Cone
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL1 Cone", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(0, 1)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 9));
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL1 Cone", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(2, 1)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 11));
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL1 Cone", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(3, 1)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 12));
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL1 Cone", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(5, 1)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 14));
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL1 Cone", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(6, 1)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 15));
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL1 Cone", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(8, 1)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 17));




        // Second Level - Cube
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL2 Cube", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(1, 0)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 19));
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL2 Cube", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(4, 0)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 22));
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL2 Cube", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(7, 0)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 25));

        // Second Level - Cone
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL2 Cone", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(0, 0)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 18));
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL2 Cone", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(2, 0)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 20));
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL2 Cone", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(3, 0)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 21));
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL2 Cone", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(5, 0)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 23));
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL2 Cone", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(6, 0)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 24));
        Shuffleboard.getTab("GamePieces")
                .add("LEVEL2 Cone", new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.Ground)))
                .withWidget(BuiltInWidgets.kCommand)
                .withPosition(8, 0)
                .withSize(1, 1)
                .withProperties(Map.of("ID", 26));
    }

}
