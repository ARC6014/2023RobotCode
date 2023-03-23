package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.Constants.GrabberConstants;
import frc.robot.RobotState.pieceState;

public class GrabberSubsystem extends SubsystemBase{

    private final TalonSRX grabberMotorMaster = new TalonSRX(GrabberConstants.grabberMotorMasterID);
    private final TalonSRX grabberMotorSlave = new TalonSRX(GrabberConstants.grabberMotorSlaveID);

    private static GrabberSubsystem m_instance;

    public GrabberSubsystem() {
        grabberMotorMaster.setNeutralMode(NeutralMode.Brake);
        grabberMotorSlave.setNeutralMode(NeutralMode.Brake);
        grabberMotorSlave.setInverted(false);
        grabberMotorMaster.setInverted(false);
        grabberMotorSlave.follow(grabberMotorMaster);

    }

    public static GrabberSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new GrabberSubsystem();
        }
        return m_instance;
    }

    public void grab() {
        if(RobotState.getInstance().getPiece() == pieceState.CONE){
            grabberMotorMaster.set(ControlMode.PercentOutput, .7);
        }else{
            grabberMotorMaster.set(ControlMode.PercentOutput, .25);
        }
    }

    public void release() {
        if(RobotState.getInstance().getPiece() == pieceState.CONE){
            grabberMotorMaster.set(ControlMode.PercentOutput, -0.1);
        }else{
            grabberMotorMaster.set(ControlMode.PercentOutput, -0.35);
        }
    }

    public void stop() {
        grabberMotorMaster.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void setOutput(double speed) {
        grabberMotorMaster.set(TalonSRXControlMode.PercentOutput, speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
