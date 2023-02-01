package frc.robot;

import frc.team6014.lib.util.SwerveUtils.SwerveConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final String CANIVORE_CANBUS = "CANivore";
    public static final int Pigeon2CanID = 0;

    public static final boolean tuningMode = false;

    public static final double wheelBaseLength = 0.59;
    private static final double WheelBaseWidth = 0.59;

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
        public static final double driveSlewRateLimitX = 2;
        public static final double driveSlewRateLimitY = 2;
        public static final double driveSlewRateLimitRot = 5;

        public static final double angleGearboxRatio = 22.93;
        public static final double driveGearboxRatio = 8.76923;
        public static final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;

        public static final double drivekP = 0.03;
        public static final double drivekI = 0;
        public static final double drivekD = 0;
        public static final double drivekS = 0.0065;
        public static final double drivekV = 0.27;
        public static final double drivekA = 0.1;

        public static final double anglekP = 0.25; 
        public static final double anglekI = 0;
        public static final double anglekD = 0.0;

        public static final double snapkP = 2.2;
        public static final double snapkI = 0.0;
        public static final double snapkD = 0.0;

        public static final double maxSpeed = 3.5;

        public static final double maxTransSpeedMetersPerSecond = 3.31;
        public static final double maxAngularSpeedRadPerSec = 2 * Math.PI;        
        public static final double maxAngularAccelRadPerSecSq = Math.pow(maxAngularSpeedRadPerSec, 2);

        public static final TrapezoidProfile.Constraints rotPIDconstraints = new TrapezoidProfile.Constraints(
            maxAngularSpeedRadPerSec, maxAngularAccelRadPerSecSq);

        public static SwerveConstants swerveConstants = SwerveConstants.generateSwerveConstants(angleContinuousCurrentLimit, 
        anglePeakCurrentLimit, anglePeakCurrentDuration, angleEnableCurrentLimit, driveContinuousCurrentLimit, 
        drivePeakCurrentLimit, drivePeakCurrentDuration, driveEnableCurrentLimit, openLoopRamp, closedLoopRamp);
    }

    public static final class AutoConstants{

    public static final double kMaxAcceleration = 2.5;
    public static final double kMaxSpeed = 2.2; 

    public static final double kMaxAccelerationOnTeleop = 2.5;
    public static final double kMaxSpeedOnTeleop = 2.2; 

    public static final double OnTheFlyPathGenerationTreshold = 0.25;

    public static final double kMaxAngularSpeed = Math.PI  /* * 1.2 */ ; 

    public static final double kMaxAngularAccel = Math.pow(kMaxAngularSpeed, 2); 

    public static final double kPXController = 0.8;
    public static final double kPYController = 0.8;
    public static final double kPThetaController = 1.6;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed, kMaxAngularAccel);

    public static final PathPoint testPoint = new PathPoint(new Translation2d(0,0), new Rotation2d(0), new Rotation2d(0));
   }

    public static final class SwerveModuleFrontLeft {
        public static final int angleMotorID = 0;
        public static final int driveMotorID = 1;
        public static final int cancoderID = 0;
        public static final double angleOffset = -74.57 - 358.24 - 359.121;
        public static final double modulekS = 0.0065;
        public static final double modulekV = 0.27;
    }

    public static final class SwerveModuleFrontRight {
        public static final int angleMotorID = 2;
        public static final int driveMotorID = 3;
        public static final int cancoderID = 1;
        public static final double angleOffset = 87.1 - 357.80 - 358.51;
        public static final double modulekS = 0.0065;
        public static final double modulekV = 0.27;
    }
    
    public static final class SwerveModuleRearLeft {
        public static final int angleMotorID = 4;
        public static final int driveMotorID = 5;
        public static final int cancoderID = 2;
        public static final double angleOffset = -95.54 - 358.51 - 358.59;
        public static final double modulekS = 0.0065;
        public static final double modulekV = 0.27;
    }
    
    public static final class SwerveModuleRearRight {
        public static final int angleMotorID = 6;
        public static final int driveMotorID = 7;
        public static final int cancoderID = 3;
        public static final double angleOffset = 66.42 - 146.69 - 1.67;
        public static final double modulekS = 0.0065;
        public static final double modulekV = 0.27;
    }

}
