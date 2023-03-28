package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.GrabberConstants;
import frc.robot.RobotState.pieceState;
import frc.robot.RobotState.scoreLevel;

public class GrabberSubsystem extends SubsystemBase{

    private final CANSparkMax grabberMotorMaster = new CANSparkMax(GrabberConstants.grabberMotorMasterID, MotorType.kBrushless);
    private final CANSparkMax grabberMotorSlave = new CANSparkMax(GrabberConstants.grabberMotorSlaveID, MotorType.kBrushless);

    private static GrabberSubsystem m_instance;

    public GrabberSubsystem() {
        grabberMotorMaster.setIdleMode(IdleMode.kBrake);
        grabberMotorSlave.setIdleMode(IdleMode.kBrake);
        grabberMotorSlave.setInverted(true);
        grabberMotorMaster.setInverted(true);
        grabberMotorSlave.follow(grabberMotorMaster, true);
        grabberMotorMaster.burnFlash();
        grabberMotorSlave.burnFlash();

    }

    public static GrabberSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new GrabberSubsystem();
        }
        return m_instance;
    }

    public void grab() {
        if(RobotState.getInstance().getPiece() == pieceState.CONE){
            grabberMotorMaster.set(-0.35);
        }else{
            grabberMotorMaster.set(-0.25);
        }
        System.out.println("alo");
    }

    public void release() {
        if(RobotState.getInstance().getPiece() == pieceState.CONE){
            if(RobotState.getInstance().getScoreTarget() == scoreLevel.FirstLevel){
                grabberMotorMaster.set(0.03);
            }
                grabberMotorMaster.set(0.1);
        }else{
            grabberMotorMaster.set(0.3);
        }
    }

    public void stop() {
        grabberMotorMaster.set(0);
    }

    public void setOutput(double speed) {
        grabberMotorMaster.set(speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
