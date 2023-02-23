// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.premiereCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team6014.SuperStructureState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SuperStructureToTarget extends SequentialCommandGroup {
  /** Creates a new SuperStructureToTarget. */

  public SuperStructureToTarget(SuperStructureState targetState) {
    final SetTelescop m_TelescopPivot = new SetTelescop(new SuperStructureState(0, 92.225, 0));
    final SetElevator m_ElevatorPivot = new SetElevator(new SuperStructureState(39.142 , 0,0));

    final SetArm m_Arm = new SetArm(targetState);
    final SetTelescop m_Telescop = new SetTelescop(targetState);
    final SetElevator m_Elevator = new SetElevator(targetState);

    final HoldArm m_ArmHolder = new HoldArm();
    final HoldElevator m_ElevatorHolder = new HoldElevator();
    final HoldTelescop m_TelescopHolder = new HoldTelescop();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        m_TelescopPivot,
        m_ElevatorPivot,
        m_ArmHolder
      ),
      new ParallelCommandGroup(
        m_Arm,
        m_ElevatorHolder,
        m_TelescopHolder
      ),
      new ParallelCommandGroup(
        m_ArmHolder,
        m_Elevator,
        m_Telescop
      )
    );
  }
}
