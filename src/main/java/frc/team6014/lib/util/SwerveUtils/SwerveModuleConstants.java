package frc.team6014.lib.util.SwerveUtils;


public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final double CANcoderangleOffset;
    public final double moduleTuningkS;
    public final double moduleTuningkV;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double CANcoderangleOffset, double moduleTuningkS, double moduleTuningkV) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.CANcoderangleOffset = CANcoderangleOffset;
        this.moduleTuningkS = moduleTuningkS;
        this.moduleTuningkV = moduleTuningkV;
    }

    public static SwerveModuleConstants generateModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double angleOffset, double moduleTuningkS, double moduleTuningkV) {
        return new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, moduleTuningkS, moduleTuningkV);
    }
}