package frc.robot;

import frc.team6014.lib.util.SwerveUtils.SwerveConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final String CANIVORE_CANBUS = "CANivore";
    public static final int Pigeon2CanID = 0;

    public static final boolean tuningMode = false;

    public static final double wheelBaseLength = 0.639;
    private static final double WheelBaseWidth = 0.639;

    public static final Translation2d FRONTLEFTMODULE_TRANSLATION2D = new Translation2d(wheelBaseLength / 2, WheelBaseWidth / 2);
    public static final Translation2d FRONTRIGHTMODULE_TRANSLATION2D = new Translation2d(wheelBaseLength / 2, -WheelBaseWidth / 2);
    public static final Translation2d REARLEFTMODULE_TRANSLATION2D = new Translation2d(-wheelBaseLength / 2, WheelBaseWidth / 2);
    public static final Translation2d REARRIGHTMODULE_TRANSLATION2D = new Translation2d(-wheelBaseLength / 2, -WheelBaseWidth / 2);

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            FRONTLEFTMODULE_TRANSLATION2D,  //FL
            FRONTRIGHTMODULE_TRANSLATION2D,  //FR
            REARLEFTMODULE_TRANSLATION2D,  //RL
            REARRIGHTMODULE_TRANSLATION2D);  //RR

    public static final class DriveConstants{
        
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

        public static final double drivePowerScalar = 0.6;
        public static final double driveSlewRateLimitX = 2.5;
        public static final double driveSlewRateLimitY = 2.5;
        public static final double driveSlewRateLimitRot = 5;

        public static final double angleGearboxRatio = 22.93;
        public static final double driveGearboxRatio = 6.59340659;
        public static final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;

        public static final double drivekP = 0.00;
        public static final double drivekI = 0;
        public static final double drivekD = 0;
        public static final double drivekS = 0.0;
        public static final double drivekV = 0.0;
        public static final double drivekA = 0.0;

        public static final double anglekP = 0.25; 
        public static final double anglekI = 0;
        public static final double anglekD = 0.0;

        public static final double snapkP = 2.2;
        public static final double snapkI = 0.0;
        public static final double snapkD = 0.0;

        public static final double maxSpeed = 5;

        public static final double maxTransSpeedMetersPerSecond = 3.3;
        public static final double maxAngularSpeedRadPerSec = 2 * Math.PI;        
        public static final double maxAngularAccelRadPerSecSq = Math.pow(maxAngularSpeedRadPerSec, 2);

        public static final TrapezoidProfile.Constraints rotPIDconstraints = new TrapezoidProfile.Constraints(
            maxAngularSpeedRadPerSec, maxAngularAccelRadPerSecSq);

        public static SwerveConstants swerveConstants = SwerveConstants.generateSwerveConstants(angleContinuousCurrentLimit, 
        anglePeakCurrentLimit, anglePeakCurrentDuration, angleEnableCurrentLimit, driveContinuousCurrentLimit, 
        drivePeakCurrentLimit, drivePeakCurrentDuration, driveEnableCurrentLimit, openLoopRamp, closedLoopRamp);
    }

    public static final class AutoConstants{

    public static final double kMaxSpeed = 4;
    public static final double kMaxAcceleration = 3.5; 
    
    public static final double kPXController = 0.85;
    public static final double kPYController = 0.85;
    public static final double kPThetaController = 1.5;

    public static final double kPdriveOnTeleop = 0.5;  
    public static final double kDdriveOnTeleop = 0.0;  
    public static final double kPturnOnTeleop = 1.0;
    public static final double kDturnOnTeleop = 0.0;
    public static final double onTheFlyMoveTreshold = 0.5;

    public static final double kMaxAngularSpeed = Math.PI * 1.2  ; 

    public static final double kMaxAngularAccel = Math.pow(kMaxAngularSpeed, 2); 

    public static final double OnTheFlyPathGenerationTreshold = 0.5;
    public static final double kMaxAccelerationOnTeleop = 2;  
    public static final double kMaxSpeedOnTeleop = 1.8; 

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed, kMaxAngularAccel);

    public static final double kPositionToleranceX = 0.25;
    public static final double kPositionToleranceY = 0.25;
    public static final double kRotationToleranceRadians = 0.125;

    public static final Pose2d testPose = new Pose2d(7.10,3.35, Rotation2d.fromDegrees(0));
    public static final Pose2d testPose2 = new Pose2d(7,7.5, Rotation2d.fromDegrees(0));
    public static final Pose2d FIRST_PIVOT_POSE2D = new Pose2d(5.5,4.75, Rotation2d.fromDegrees(0));
    public static final Pose2d SECOND_PIVOT_POSE2D = new Pose2d(2.5,4.75, Rotation2d.fromDegrees(0));

    public static final Pose2d firstNode = new Pose2d(1.78,3.30, Rotation2d.fromDegrees(0));
    public static final Pose2d firstCube = new Pose2d(1.78,1.05, Rotation2d.fromDegrees(0));
    public static final Pose2d secondNode = new Pose2d(1.78,4.98, Rotation2d.fromDegrees(0));
    public static final Pose2d thirdNode = new Pose2d(1.78,3.85, Rotation2d.fromDegrees(0));
    public static final Pose2d secondCube = new Pose2d(1.78,2.75, Rotation2d.fromDegrees(0));
    public static final Pose2d fourthNode = new Pose2d(1.78,1.64, Rotation2d.fromDegrees(0));
    public static final Pose2d fifthNode = new Pose2d(1.78,4.40, Rotation2d.fromDegrees(0));
    public static final Pose2d thirdCube = new Pose2d(1.78,2.18, Rotation2d.fromDegrees(0));
    public static final Pose2d sixthNode = new Pose2d(1.78,0.50, Rotation2d.fromDegrees(0));
    public static final Pose2d loadingZoneCone = new Pose2d(13.45,7.00, Rotation2d.fromDegrees(0));
    public static final Pose2d loadingZoneCube = new Pose2d(13.45,7.35, Rotation2d.fromDegrees(0));
    public static final Pose2d chargingStation = new Pose2d(5.30,3.60, Rotation2d.fromDegrees(0));
   }

    public static final class SwerveModuleFrontLeft {
        public static final int angleMotorID = 0;
        public static final int driveMotorID = 1;
        public static final int cancoderID = 0;
        public static final double angleOffset = -71.931;
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }

    public static final class SwerveModuleFrontRight {
        public static final int angleMotorID = 2;
        public static final int driveMotorID = 3;
        public static final int cancoderID = 1;
        public static final double angleOffset = -269.21;
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }
    
    public static final class SwerveModuleRearLeft {
        public static final int angleMotorID = 4;
        public static final int driveMotorID = 5;
        public static final int cancoderID = 2;
        public static final double angleOffset = -92.64;
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }
    
    public static final class SwerveModuleRearRight {
        public static final int angleMotorID = 6;
        public static final int driveMotorID = 7;
        public static final int cancoderID = 3;
        public static final double angleOffset = -81.94;
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }

}
