// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.team6014.ARCTrajectoryGenerator;
import frc.team6014.lib.Pathplanner.PathPoint;
import frc.team6014.lib.drivers.SwerveModuleBase;
import frc.team6014.lib.math.AllianceFlipUtil;
import frc.team6014.lib.util.SwerveUtils.SwerveModuleConstants;

public class DriveSubsystem extends SubsystemBase{

    /*
     * private final Field2d m_field = new Field2d();
     */

    private final Trigger brakeModeTrigger;
    private final StartEndCommand brakeModeCommand;

    private static DriveSubsystem m_instance;

    public SwerveModuleBase[] m_swerveModules;
    private SwerveModuleState[] states;
    private ChassisSpeeds desiredChassisSpeeds;

    public SwerveDriveOdometry m_odometry;

    private double[] velocityDesired = new double[4];
    private double[] angleDesired = new double[4];

    Pigeon2 m_gyro = new Pigeon2(Constants.Pigeon2CanID, Constants.CANIVORE_CANBUS);

    private double snapAngle = 0.0;
    private double timeSinceRot = 0.0;
    private double lastRotTime = 0.0;
    private double timeSinceDrive = 0.0;
    private double lastDriveTime = 0.0;
    private boolean islocked = false;

    private ProfiledPIDController snapPIDController = new ProfiledPIDController(DriveConstants.snapkP,
            DriveConstants.snapkI, DriveConstants.snapkD, DriveConstants.rotPIDconstraints);

    private final Timer snapTimer = new Timer();

    public DriveSubsystem() {

        // SmartDashboard.putData("Field", m_field);

        m_swerveModules = new SwerveModuleBase[] {
                new SwerveModuleBase("FL", SwerveModuleConstants.generateModuleConstants(
                        Constants.SwerveModuleFrontLeft.driveMotorID, Constants.SwerveModuleFrontLeft.angleMotorID,
                        Constants.SwerveModuleFrontLeft.cancoderID, Constants.SwerveModuleFrontLeft.angleOffset,
                        Constants.SwerveModuleFrontLeft.modulekS, Constants.SwerveModuleFrontLeft.modulekV),
                        DriveConstants.swerveConstants),

                new SwerveModuleBase("FR", SwerveModuleConstants.generateModuleConstants(
                        Constants.SwerveModuleFrontRight.driveMotorID, Constants.SwerveModuleFrontRight.angleMotorID,
                        Constants.SwerveModuleFrontRight.cancoderID, Constants.SwerveModuleFrontRight.angleOffset,
                        Constants.SwerveModuleFrontRight.modulekS, Constants.SwerveModuleFrontRight.modulekV),
                        DriveConstants.swerveConstants),

                new SwerveModuleBase("RL", SwerveModuleConstants.generateModuleConstants(
                        Constants.SwerveModuleRearLeft.driveMotorID, Constants.SwerveModuleRearLeft.angleMotorID,
                        Constants.SwerveModuleRearLeft.cancoderID, Constants.SwerveModuleRearLeft.angleOffset,
                        Constants.SwerveModuleRearLeft.modulekS, Constants.SwerveModuleRearLeft.modulekV),
                        DriveConstants.swerveConstants),

                new SwerveModuleBase("RR", SwerveModuleConstants.generateModuleConstants(
                        Constants.SwerveModuleRearRight.driveMotorID, Constants.SwerveModuleRearRight.angleMotorID,
                        Constants.SwerveModuleRearRight.cancoderID, Constants.SwerveModuleRearRight.angleOffset,
                        Constants.SwerveModuleRearRight.modulekS, Constants.SwerveModuleRearRight.modulekV),
                        DriveConstants.swerveConstants)
        };

        snapTimer.reset();
        snapTimer.start();

        snapPIDController.enableContinuousInput(-Math.PI, Math.PI);

        zeroHeading();

        m_odometry = new SwerveDriveOdometry(Constants.kinematics, getRotation2d(), getModulePositions());

        brakeModeTrigger = new Trigger(RobotState::isEnabled);

        brakeModeCommand = new StartEndCommand(() -> {
            for (SwerveModuleBase mod : m_swerveModules) {
                mod.setNeutralMode2Brake(true);
            }
        }, () -> {
            Timer.delay(2);
            for (SwerveModuleBase mod : m_swerveModules) {
                mod.setNeutralMode2Brake(false);
            }
        });

    }

    public static DriveSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new DriveSubsystem();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        updateOdometry();

         for (SwerveModuleBase mod : m_swerveModules) {
            SmartDashboard.putNumber(mod.getName() + " - Velocity : ", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber(mod.getName() + " - Angle : ", mod.getCANCoderRotation().getDegrees());
            SmartDashboard.putNumber(mod.getName() + " - AngleFalcon : ", mod.getState().angle.getDegrees());
        }

        SmartDashboard.putNumber("Gyro : ", getRotation2d().getDegrees());
       /*  SmartDashboard.putNumber("x", getPose().getX());
        SmartDashboard.putNumber("Y", getPose().getY());*/


        brakeModeTrigger.whileTrue(brakeModeCommand);

    }

    /*
     * Manual Swerve Drive Method
     */

    public void swerveDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        rot = calculateSnapValue(xSpeed, ySpeed, rot);

        desiredChassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getDriverCentericRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot);

        states = Constants.kinematics.toSwerveModuleStates(desiredChassisSpeeds);

        if (islocked) {
            states = new SwerveModuleState[] {
                    new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
                    new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
                    new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
            };
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxSpeed);

        for (int i = 0; i < 4; i++) {
            m_swerveModules[i].setDesiredState(states[i], true);
            velocityDesired[i] = states[i].speedMetersPerSecond;
            angleDesired[i] = states[i].angle.getDegrees();
        }

    }

    public void angleAlignDrive(double xSpeed, double ySpeed, boolean fieldRelative,
            double targetHeading) {
        double rotation = snapPIDController.calculate(getRotation2d().getRadians(), Math.toRadians(targetHeading));
        swerveDrive(xSpeed, ySpeed, rotation, fieldRelative);
    }

    /*
     * Auto Swerve States Method
     */

    public synchronized void setClosedLoopStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeed);
        if (islocked) {
            states = new SwerveModuleState[] {
                    new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
                    new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
                    new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
            };
        }
        m_swerveModules[0].setDesiredState(desiredStates[0], false);
        m_swerveModules[1].setDesiredState(desiredStates[1], false);
        m_swerveModules[2].setDesiredState(desiredStates[2], false);
        m_swerveModules[3].setDesiredState(desiredStates[3], false);

    }

    public void setClosedLoopStates(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredStates = Constants.kinematics.toSwerveModuleStates(speeds);
        setClosedLoopStates(desiredStates);
    }

    public void calibrate() {
        for (SwerveModuleBase mod : m_swerveModules) {
            mod.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true);
        }
    }

    public void resetOdometry(Pose2d pose) {
        m_gyro.reset();
        m_gyro.setYaw(pose.getRotation().times(DriveConstants.invertGyro ? -1 : 1).getDegrees());
        m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
    }

    public void resetOdometry(Rotation2d angle) {
        Pose2d pose = new Pose2d(getPose().getTranslation(), angle);
        m_gyro.reset();
        m_gyro.setYaw(angle.getDegrees());
        m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
    }

    public void resetSnapPID() {
        snapPIDController.reset(getRotation2d().getRadians());
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public void stop() {
        for (SwerveModuleBase module : m_swerveModules) {
            module.stop();
        }
    }

    public void resetToAbsolute() {
        for (SwerveModuleBase module : m_swerveModules) {
            module.stop();
            module.resetToAbsolute();
        }
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(m_gyro.getAngle(), 360.0))
                .times(DriveConstants.invertGyro ? -1 : 1);
    }

    public Rotation2d getDriverCentericRotation2d() {
        return DriverStation.getAlliance() == Alliance.Red
                ? Rotation2d.fromDegrees(Math.IEEEremainder(m_gyro.getAngle() + 180, 360.0))
                        .times(DriveConstants.invertGyro ? -1 : 1)
                : Rotation2d.fromDegrees(Math.IEEEremainder(m_gyro.getAngle(), 360.0))
                        .times(DriveConstants.invertGyro ? -1 : 1);
    }

    public void lockSwerve(boolean should) {
        islocked = should;
    }

    public void updateOdometry() {
        m_odometry.update(
                getRotation2d(),
                getModulePositions());
    }

    public void setPose(Pose2d pose) {
        m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public PathPoint getPathPoint(Pose2d targetPose2d) {
        return DriverStation.getAlliance() == Alliance.Blue
                ? new PathPoint(new Translation2d(getPose().getX(), getPose().getY()), Rotation2d.fromDegrees(
                        ARCTrajectoryGenerator.getHeadingforPoints(getPose(), targetPose2d) - 90), getRotation2d())
                : new PathPoint(new Translation2d(getPose().getX(), getPose().getY()), Rotation2d.fromDegrees(
                        -ARCTrajectoryGenerator.getHeadingforPoints(getPose(), AllianceFlipUtil.apply(targetPose2d))
                                - 90),
                        getRotation2d());
    }

    SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                m_swerveModules[0].getPosition(),
                m_swerveModules[1].getPosition(),
                m_swerveModules[2].getPosition(),
                m_swerveModules[3].getPosition(),
        };
    }

    public ChassisSpeeds getChassisSpeed() {
        return Constants.kinematics.toChassisSpeeds(m_swerveModules[0].getState(), m_swerveModules[1].getState(),
                m_swerveModules[2].getState(), m_swerveModules[3].getState());
    }

    private double calculateSnapValue(double xSpeed, double ySpeed, double rot) {

        double output = rot;

        if (Math.abs(rot) >= 0.05) {
            lastRotTime = snapTimer.get();
        }

        if (Math.abs(xSpeed) >= 0.05 || Math.abs(ySpeed) >= 0.05) {
            lastDriveTime = snapTimer.get();
        }

        timeSinceRot = snapTimer.get() - lastRotTime;
        timeSinceDrive = snapTimer.get() - lastDriveTime;

        if (timeSinceRot < 0.5) {
            snapAngle = getRotation2d().getRadians();
        } else if (Math.abs(rot) < 0.05 && timeSinceDrive < 0.25) {
            output = snapPIDController.calculate(getRotation2d().getRadians(), snapAngle);
        }
        return output;
    }

    /*
     * Velocity-Angle Displays for Shuffleboard
     */
/* 
    @Log(name = "FL-Des-Vel", rowIndex = 0, columnIndex = 0)
    public double getFLDesiredVelocity() {
        return velocityDesired[0];
    }

    @Log(name = "FR-Des-Vel", rowIndex = 0, columnIndex = 1)
    public double getFRDesiredVelocity() {
        return velocityDesired[1];
    }

    @Log(name = "RL-Des-Vel", rowIndex = 1, columnIndex = 0)
    public double getRLDesiredVelocity() {
        return velocityDesired[2];
    }

    @Log(name = "RR-Des-Vel", rowIndex = 1, columnIndex = 1)
    public double getRRDesiredVelocity() {
        return velocityDesired[3];
    }

    @Log(name = "FL-Des-Ang", rowIndex = 2, columnIndex = 0)
    public double getFLDesiredAngle() {
        return angleDesired[0];
    }

    @Log(name = "FR-Des-Ang", rowIndex = 2, columnIndex = 1)
    public double getFRDesiredAngle() {
        return angleDesired[1];
    }

    @Log(name = "RL-Des-Ang", rowIndex = 3, columnIndex = 0)
    public double getRLDesiredAngle() {
        return angleDesired[2];
    }

    @Log(name = "RR-Des-Ang", rowIndex = 3, columnIndex = 1)
    public double getRRDesiredAngle() {
        return angleDesired[3];
    }*/
}
