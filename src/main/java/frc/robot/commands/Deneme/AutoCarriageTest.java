package frc.robot.commands.Deneme;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CarriageSubsystem;
import frc.team6014.SuperStructureState;

public class AutoCarriageTest extends CommandBase {

    private CarriageSubsystem m_carriage = CarriageSubsystem.getInstance();
    private SuperStructureState m_targetState = new SuperStructureState(120,95, 40);
    private DoubleSupplier m_outputSupplier;

    public AutoCarriageTest(DoubleSupplier output) {
        m_outputSupplier = output;
        addRequirements(m_carriage);
    }

@Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = m_outputSupplier.getAsDouble();
    System.out.println(output);
    output = Double.min(output, 1);
    output = Double.max(-1, output);
    System.out.println(output / 2);
    m_carriage.setMotorOutput(output/2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_carriage.holdCarriagePosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
