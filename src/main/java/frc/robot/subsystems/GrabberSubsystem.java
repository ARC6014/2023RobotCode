package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;
import frc.team6014.lib.drivers.TalonSRXFactory;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class GrabberSubsystem extends SubsystemBase{

    private TalonSRX grabberMotorMaster;
    private TalonSRX grabberMotorSlave;

    private static GrabberSubsystem m_instance;

   // @Log
   // @Config
    private double output;

    public GrabberSubsystem() {
        grabberMotorMaster = new TalonSRX(GrabberConstants.grabberMotorMasterID);
        grabberMotorSlave = new TalonSRX(GrabberConstants.grabberMotorSlaveID);

        grabberMotorSlave.setInverted(false);
        grabberMotorMaster.setInverted(false);
        grabberMotorSlave.follow(grabberMotorMaster);
        // indexSensor = new IndexingSensor(Constants.GrabberConstants.sensorChannel,
        // Constants.GrabberConstants.kDistanceConstant);

        output = GrabberConstants.kMaxOutput * (2 / 3);
    }

    public static GrabberSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new GrabberSubsystem();
        }
        return m_instance;
    }

    // percent output
    public void grab() {
        grabberMotorMaster.set(ControlMode.PercentOutput, 0.7);
    }

    public void release() {
        grabberMotorMaster.set(ControlMode.PercentOutput, -0.15);
    }

    public void stop() {
        grabberMotorMaster.set(TalonSRXControlMode.PercentOutput, 0);
    }

  //  @Log.NumberBar(name = "Motor Speed", columnIndex = 0, rowIndex = 3)
    public double getSpeed() {
        return grabberMotorMaster.getMotorOutputPercent();
    }

    public void setOutput(double speed) {
        this.output = speed;
        grabberMotorMaster.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public double getRPM() {
        return grabberMotorMaster.getSelectedSensorVelocity(0);
    }

    /*
     * @Log.BooleanBox(name = "index", columnIndex = 0, rowIndex = 0)
     * public boolean isIndexed() {
     * return indexSensor.isThereAThing();
     * }
     */

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
