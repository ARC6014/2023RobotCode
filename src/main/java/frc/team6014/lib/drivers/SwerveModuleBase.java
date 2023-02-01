package frc.team6014.lib.drivers;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.math.Gearbox;
import frc.team6014.lib.util.SwerveUtils.CTREConfigs;
import frc.team6014.lib.util.SwerveUtils.CTREModuleState;
import frc.team6014.lib.util.SwerveUtils.SwerveConstants;
import frc.team6014.lib.util.SwerveUtils.SwerveModuleConstants;


public class SwerveModuleBase {
    
    private String m_ID;
    private WPI_TalonFX m_driveMotor;
    private WPI_TalonFX m_angleMotor;

    private WPI_CANCoder m_rotEncoder;

    private SimpleMotorFeedforward m_driveFF;

    private boolean isDriveMotorInverted = false;
    private boolean isAngleMotorInverted = true;
    private boolean isRotEncoderInverted = false;
    
    private Gearbox driveGearbox = new Gearbox(DriveConstants.driveGearboxRatio);
    private Gearbox angleGearbox = new Gearbox(DriveConstants.angleGearboxRatio);

    private double m_wheelCircumference = DriveConstants.wheelCircumference;

    private double maxSpeed = DriveConstants.maxSpeed;

    private double lastAngle;

    /**
     * @param name  Name of the Module
     * @param constants     Individual constants for a module
     * @param swerveConstants   Global swerve base constants
     * @param feedForward   feed forward of the swerve system
     */

    public SwerveModuleBase(
        String name, SwerveModuleConstants constants, SwerveConstants swerveConstants) {

        m_ID = name;

        m_driveFF = new SimpleMotorFeedforward(constants.moduleTuningkS, constants.moduleTuningkV, DriveConstants.drivekA);

        m_rotEncoder = new WPI_CANCoder(constants.cancoderID, Constants.CANIVORE_CANBUS);
        m_driveMotor = new WPI_TalonFX(constants.driveMotorID);
        m_angleMotor = new WPI_TalonFX(constants.angleMotorID);

        configAll();

        m_rotEncoder.configMagnetOffset(constants.CANcoderangleOffset);

        resetToAbsolute();

        lastAngle = getCANCoderRotation().getDegrees();

    }


    /* 
     *Set Methods
     */

    public void stop(){
        m_driveMotor.set(.0);
        m_angleMotor.set(.0);
    }

    public synchronized void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / maxSpeed;
            m_driveMotor.set(TalonFXControlMode.PercentOutput, percentOutput);   
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, m_wheelCircumference, driveGearbox.getRatio());
            m_driveMotor.set(TalonFXControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, m_driveFF.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees();

        
        m_angleMotor.set(TalonFXControlMode.Position, Conversions.degreesToFalcon(lastAngle , angleGearbox.getRatio()));

        lastAngle = angle;
                
    }

    public void setNeutralMode2Brake(boolean brake){
        m_driveMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /*
     * Config Methods
     */

    private void configRotEncoder(){        
        m_rotEncoder.configFactoryDefault();
        m_rotEncoder = CTREConfigs.swerveCancoderConfig(m_rotEncoder);
        m_rotEncoder.configSensorDirection(isRotEncoderInverted);
    }

    private void configAngleMotor(){
        m_angleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 20);
        m_angleMotor = CTREConfigs.swerveAngleFXConfig(m_angleMotor);
        m_angleMotor.setInverted(isAngleMotorInverted);
    }

    private void configDriveMotor(){        
        m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 20);
        m_driveMotor = CTREConfigs.swerveDriveFXConfig(m_driveMotor);
        m_driveMotor.setInverted(isDriveMotorInverted);
    }
    public void configAll() {
        configRotEncoder();
        configDriveMotor();
        configAngleMotor();
    }

    public void resetToAbsolute(){
        double position = Conversions.degreesToFalcon(getCANCoderRotation().getDegrees(), angleGearbox.getRatio());
        m_angleMotor.setSelectedSensorPosition(position);
    }

    
    /*
     * Get Methods
     */

    public WPI_TalonFX getDriveMotor(){
        return m_driveMotor;
    }

    public WPI_TalonFX getAngleMotor(){
        return m_angleMotor;
    }

    public String getName() {
        return m_ID;
    }

    public Rotation2d getCANCoderRotation() {
        return Rotation2d.fromDegrees(m_rotEncoder.getAbsolutePosition());
    }

    public SwerveModulePosition getPosition(){
        double position = Conversions.FalconToRotation(m_driveMotor.getSelectedSensorPosition(), driveGearbox.getRatio()) * m_wheelCircumference;
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), angleGearbox.getRatio()));
        return new SwerveModulePosition(position, angle);
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), m_wheelCircumference, driveGearbox.getRatio());
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), angleGearbox.getRatio()));
        return new SwerveModuleState(velocity, angle);
    }
}
