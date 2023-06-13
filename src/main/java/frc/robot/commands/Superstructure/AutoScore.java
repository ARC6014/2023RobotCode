// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.RobotState.scoreLevel;
import frc.robot.commands.Grabbing.RelaseCommand;
import frc.team6014.MoveToPose;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScore extends SequentialCommandGroup {
  /** Creates a new AutoScore. */
  public AutoScore() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if(RobotState.getInstance().getScoreTarget() != scoreLevel.Intake /*&& RobotState.getInstance().getScoreTarget() != scoreLevel.HOMING*/){
      addCommands(
        //new MoveToPose(() -> RobotState.getInstance().getTargetPose()),
        new SmartMotion().withTimeout(2),
        //new WaitCommand(0.5),
        //new MoveToPose(() -> RobotState.getInstance().getScorePose()),
        new WaitCommand(0.2),
        new RelaseCommand().withTimeout(0.5),
        new InstantCommand(() -> RobotState.getInstance().setScoreLevel(scoreLevel.HOMING)),
        //new MoveToPose(() -> RobotState.getInstance().getTargetPose()),
        new SmartMotion()
      );
    }
    addCommands(
      
    );

  }
}
