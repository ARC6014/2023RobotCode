// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveByJoystick extends CommandBase {

  private final DriveSubsystem m_drive = DriveSubsystem.getInstance();
  private final DoubleSupplier m_x;
  private final DoubleSupplier m_y;
  private final DoubleSupplier m_rotation;
  private final BooleanSupplier m_isLocked;
  private final BooleanSupplier m_rush;

  private final SlewRateLimiter m_slewX = new SlewRateLimiter(DriveConstants.driveSlewRateLimitX);
  private final SlewRateLimiter m_slewY = new SlewRateLimiter(DriveConstants.driveSlewRateLimitY);
  private final SlewRateLimiter m_slewRot = new SlewRateLimiter(DriveConstants.driveSlewRateLimitRot);

  private boolean fieldOrient = DriveConstants.isFieldOriented;
  private double scalarValue = DriveConstants.drivePowerScalar;

  public DriveByJoystick(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, BooleanSupplier isLocked, BooleanSupplier rush) {
    m_x=x;
    m_y=y;
    m_rotation = rotation;
    m_isLocked=isLocked;
    m_rush = rush;
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_drive.lockSwerve(m_isLocked.getAsBoolean());

    double scalar = m_rush.getAsBoolean()? 1 : scalarValue;   

    double xSpeed = m_slewX.calculate(inputTransform(m_x.getAsDouble()) * DriveConstants.maxSpeed) * scalar;
    double ySpeed = m_slewY.calculate(inputTransform(m_y.getAsDouble()) * DriveConstants.maxSpeed) * scalar;
    double rotation = m_slewRot.calculate(inputTransform(m_rotation.getAsDouble()) * DriveConstants.maxAngularSpeedRadPerSec) * scalar;

    if(m_isLocked.getAsBoolean()){
      m_slewX.reset(0);
      m_slewY.reset(0);
      m_slewRot.reset(0);
      m_drive.resetSnapPID();
    }

    m_drive.swerveDrive(xSpeed, ySpeed, rotation, fieldOrient);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double inputTransform(double input) {
    if(input < 0){
      return -Math.pow(input, 2);
    }else{
      return Math.pow(input, 2);
    }
  }

}
