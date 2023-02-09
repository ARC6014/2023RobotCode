// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team6014.MoveToPose;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopMoveToPose extends SequentialCommandGroup {
  /** Creates a new TeleopMoveToPose. */
  public TeleopMoveToPose(Pose2d firtPose, Pose2d secondPose, Pose2d thirdPose) {
    // Add your commands in the addCommands() call, e.g.
    final MoveToPose getFirstPivotPoint = new MoveToPose(firtPose, false);
    final MoveToPose getSeconPivotPoint = new MoveToPose(secondPose, false);
    final MoveToPose getTargetPose = new MoveToPose(thirdPose, true);
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> SmartDashboard.putString("Target Pose : ", thirdPose.toString())),
      getFirstPivotPoint,
      getSeconPivotPoint,
      getTargetPose
    );
    
  }
}
