package frc.robot;

import frc.team6014.lib.util.SwerveUtils.SwerveConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final String CANIVORE_CANBUS = "CANivore";
    public static final int Pigeon2CanID = 0;

    public static final boolean tuningMode = true;

    public static final double wheelBaseLength = 0.639;
    private static final double WheelBaseWidth = 0.639;

    public static final Translation2d FRONTLEFTMODULE_TRANSLATION2D = new Translation2d(wheelBaseLength / 2,
            WheelBaseWidth / 2);
    public static final Translation2d FRONTRIGHTMODULE_TRANSLATION2D = new Translation2d(wheelBaseLength / 2,
            -WheelBaseWidth / 2);
    public static final Translation2d REARLEFTMODULE_TRANSLATION2D = new Translation2d(-wheelBaseLength / 2,
            WheelBaseWidth / 2);
    public static final Translation2d REARRIGHTMODULE_TRANSLATION2D = new Translation2d(-wheelBaseLength / 2,
            -WheelBaseWidth / 2);

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            FRONTLEFTMODULE_TRANSLATION2D, // FL
            FRONTRIGHTMODULE_TRANSLATION2D, // FR
            REARLEFTMODULE_TRANSLATION2D, // RL
            REARRIGHTMODULE_TRANSLATION2D); // RR

    public static final class DriveConstants {

        public static final boolean isFieldOriented = true;
        public static final boolean invertGyro = true; // CCW must be positive

        public static final int angleContinuousCurrentLimit = 20;
        public static final int anglePeakCurrentLimit = 35;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.2;
        public static final boolean driveEnableCurrentLimit = true;

        public static final NeutralMode angleMotorNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveMotorNeutralMode = NeutralMode.Brake;

        public static final double openLoopRamp = 0;
        public static final double closedLoopRamp = 0;

        public static final double drivePowerScalar = 0.5;
        public static final double driveSlewRateLimitX = 7;
        public static final double driveSlewRateLimitY = 7;
        public static final double driveSlewRateLimitRot = 12;

        public static final double angleGearboxRatio = 22.93;
        public static final double driveGearboxRatio = 6.59340659;
        public static final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;

        public static final double drivekP = 0.05;
        public static final double drivekI = 0;
        public static final double drivekD = 0;
        public static final double drivekS = 0.016;
        public static final double drivekV = 0.19;
        public static final double drivekA = 0.0;

        public static final double anglekP = 0.27;
        public static final double anglekI = 0;
        public static final double anglekD = 0.0;

        public static final double snapkP = 2.5;
        public static final double snapkI = 0.0;
        public static final double snapkD = 0.01;

        public static final double maxSpeed = 5;

        public static final double maxTransSpeedMetersPerSecond = 3.3;
        public static final double maxAngularSpeedRadPerSec = 2 * Math.PI;
        public static final double maxAngularAccelRadPerSecSq = Math.pow(maxAngularSpeedRadPerSec, 2);

        public static final TrapezoidProfile.Constraints rotPIDconstraints = new TrapezoidProfile.Constraints(
                maxAngularSpeedRadPerSec, maxAngularAccelRadPerSecSq);

        public static SwerveConstants swerveConstants = SwerveConstants.generateSwerveConstants(
                angleContinuousCurrentLimit,
                anglePeakCurrentLimit, anglePeakCurrentDuration, angleEnableCurrentLimit, driveContinuousCurrentLimit,
                drivePeakCurrentLimit, drivePeakCurrentDuration, driveEnableCurrentLimit, openLoopRamp, closedLoopRamp);
    }

    public static final class AutoConstants {

        public static final double kMaxSpeed = 2;
        public static final double kMaxAcceleration = 2.2;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 4;

        public static final double kPdriveOnTeleop = 3;
        public static final double kDdriveOnTeleop = 0.001;
        public static final double kPturnOnTeleop = 5;
        public static final double kDturnOnTeleop = 0.015;
        public static final double onTheFlyMoveTreshold = 0.5;

        public static final double kMaxAngularSpeed = Math.PI;

        public static final double kMaxAngularAccel = Math.pow(kMaxAngularSpeed, 2.25);

        public static final double OnTheFlyPathGenerationTreshold = 0.5;
        public static final double kMaxAccelerationOnTeleop = 6.5;
        public static final double kMaxSpeedOnTeleop = 5;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeed, kMaxAngularAccel);

        public static final TrapezoidProfile.Constraints kTranslationConstraints = new TrapezoidProfile.Constraints(
            kMaxSpeedOnTeleop, kMaxAccelerationOnTeleop);

        public static final double kPositionToleranceX = 0.05;
        public static final double kPositionToleranceY = 0.05;
        public static final double kRotationToleranceRadians = 0.1;

        public static final Pose2d testPose = new Pose2d(7.10, 3.35, Rotation2d.fromDegrees(0));
        public static final Pose2d testPose2 = new Pose2d(7, 7.5, Rotation2d.fromDegrees(0));
        public static final Pose2d FIRST_PIVOT_POSE2D = new Pose2d(5.5, 4.75, Rotation2d.fromDegrees(0));
        public static final Pose2d SECOND_PIVOT_POSE2D = new Pose2d(2.5, 4.75, Rotation2d.fromDegrees(0));

        public static final Pose2d firstNode = new Pose2d(1.78, 3.30, Rotation2d.fromDegrees(0));
        public static final Pose2d firstCube = new Pose2d(1.78, 1.05, Rotation2d.fromDegrees(0));
        public static final Pose2d secondNode = new Pose2d(1.78, 4.98, Rotation2d.fromDegrees(0));
        public static final Pose2d thirdNode = new Pose2d(1.78, 3.85, Rotation2d.fromDegrees(0));
        public static final Pose2d secondCube = new Pose2d(1.78, 2.75, Rotation2d.fromDegrees(0));
        public static final Pose2d fourthNode = new Pose2d(1.78, 1.64, Rotation2d.fromDegrees(0));
        public static final Pose2d fifthNode = new Pose2d(1.78, 4.40, Rotation2d.fromDegrees(0));
        public static final Pose2d thirdCube = new Pose2d(1.78, 2.18, Rotation2d.fromDegrees(0));
        public static final Pose2d sixthNode = new Pose2d(1.78, 0.50, Rotation2d.fromDegrees(0));
        public static final Pose2d loadingZoneCone = new Pose2d(13.45, 7.00, Rotation2d.fromDegrees(0));
        public static final Pose2d loadingZoneCube = new Pose2d(13.45, 7.35, Rotation2d.fromDegrees(0));
        public static final Pose2d chargingStation = new Pose2d(5.30, 3.60, Rotation2d.fromDegrees(0));
    }

    public static final class SwerveModuleFrontLeft {
        public static final int angleMotorID = 0;
        public static final int driveMotorID = 1;
        public static final int cancoderID = 0;
        public static final double angleOffset = -66.9863 + 4.8;
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }

    public static final class SwerveModuleFrontRight {
        public static final int angleMotorID = 2;
        public static final int driveMotorID = 3;
        public static final int cancoderID = 1;
        public static final double angleOffset = -192.91 - 6.32 + 9.29;
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }

    public static final class SwerveModuleRearLeft {
        public static final int angleMotorID = 4;
        public static final int driveMotorID = 5;
        public static final int cancoderID = 2;
        public static final double angleOffset = -112.244 - 5.36 + 4.8;
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }

    public static final class SwerveModuleRearRight {
        public static final int angleMotorID = 6;
        public static final int driveMotorID = 7;
        public static final int cancoderID = 3;
        public static final double angleOffset = -60.8375 - 3.51 - 5.69;
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }

    public static final class GrabberConstants {
        public static int grabberMotorMasterID = 20;
        public static int grabberMotorSlaveID = 21;


        public static double kMinOutput = -1.0;
        public static double kMaxOutput = 1.0;
        public static double kMaxRPM = 5700;
        public static double kP = 0.1;
        public static double kI = 0.01;
        public static double kD = 0.1;
        public static double kIz = 0;

        public static double kFF = 0.0;
        public static double gearRatio = 5.0;

        public static int sensorChannel = 2;
        public static double kDistanceConstant = 0.5;
        public static int smartCurrentLimit = 40;
        public static double kS = 0.0;
        public static double kV = 0.0;
        public static double kA = 0.0;
        public static double rpm = 0.5;
    }

    public static final class IntakeConstants {
        public static final int intakeMotorID = 10;
        public static final int intakeDoubleSolenoidPort1 = 0;
        public static final int intakeDoubleSolenoidPort2 = 1;

        public static final double intakeSpeed = 0.45;
        public static final double outtakeSpeed = -0.3;
        public static final boolean isIntakeInverted = true;
        public static final CANSparkMax.IdleMode idleMode = CANSparkMax.IdleMode.kCoast;

        // PID
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        // Limits
        public static final int stallCurrentLimit = 60;
        public static final int freeCurrentLimit = 35;
        public static final int kMaxRPM = 5700;
        public static final double RPM = 0;
        
        public static final double kIntakeTolerance = 0;
    }

    public static final class CarriageConstants {
        public static final int encoderID = 0;
        public static final int carriageMasterID = 40;

        public static final int encoderDrivingGear = 10;
        public static final int encoderDrivenGear = 40;

        public static final double kP = 3.65;
        public static final double kI = 0;
        public static final double kD = 0.45;
        public static final double kS = 0;
        public static final double kV = 0;

        public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public static final InvertedValue invertedValue = InvertedValue.Clockwise_Positive;
        public static final double peakForwardVoltage = 8;
        public static final double peakReverseVoltage = -8;

        public static final double peakForwardTorqueCurrent = 200;
        public static final double peakReverseTorqueCurrent = 200;

        public static final double dutyCycleNeutralDeadband = 0.04;

        public static final double statorCurrentLimit = 300;
        public static final boolean statorCurrentLimitEnable = true;
        public static final double supplyCurrentLimit = 80;
        public static final boolean supplyCurrentLimitEnable = true;

    }

    public static final class TelescobicArmConstants {

        public static final int telesobicMotorID = 50;

        public static final double kP = 3.65;
        public static final double kI = 0;
        public static final double kD = 0.45;
        public static final double kS = 0;
        public static final double kV = 0;

        public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public static final InvertedValue invertedValue = InvertedValue.Clockwise_Positive;
        public static final double peakForwardVoltage = 8;
        public static final double peakReverseVoltage = -8;

        public static final double peakForwardTorqueCurrent = 200;
        public static final double peakReverseTorqueCurrent = 200;

        public static final double dutyCycleNeutralDeadband = 0.04;

        public static final double statorCurrentLimit = 300;
        public static final boolean statorCurrentLimitEnable = true;
        public static final double supplyCurrentLimit = 80;
        public static final boolean supplyCurrentLimitEnable = true;

    }

    public static final class JoystickButtonConstants{

        public static final int square = 1;
        public static final int X = 2;
        public static final int O = 3;
        public static final int triangle = 4;
        public static final int L1 = 5;
        public static final int R1 = 6;
        public static final int L2 = 7;
        public static final int R2 = 8;
        public static final int createButton = 9;
        public static final int optionsButton = 10;
        public static final int L3 = 11;
        public static final int R3 = 12;
        public static final int dne = 13;
        public static final int touchpadButton = 14;
        public static final int muteButton = 15;

        

    }

    public static final class AddressableLedConstants {
        public static final int ledPort = 0; // TODO: Config PWM Port
        public static final int ledLength = 15; // 15 leds will be lit
    }

}
