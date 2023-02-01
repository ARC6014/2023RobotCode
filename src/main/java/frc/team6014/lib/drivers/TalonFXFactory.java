package frc.team6014.lib.drivers;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

public class TalonFXFactory {

    private final static int TIMEOUT_MS = 100;
    public static class Configuration {
        public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        // factory default values
        public double NEUTRAL_DEADBAND = 0.04;

        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_SOFT_LIMIT = false;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public int FORWARD_SOFT_LIMIT = 0;
        public int REVERSE_SOFT_LIMIT = 0;

        public boolean INVERTED = false;
        public boolean SENSOR_PHASE = false;

        public int CONTROL_FRAME_PERIOD_MS = 20; // 10
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 20; // 10
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

        public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(false, 150,
                350, 1);
        public SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(false, 25,
                80, 1);

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;
    } 

    private static Configuration kDefaultConfiguration = new Configuration();

    public static WPI_TalonFX createDefaultTalonFX(int id) {
        return generateTalonFX(id, kDefaultConfiguration);
    }

    public static WPI_TalonFX generateTalonFX(int id, Configuration config) {
        WPI_TalonFX newTalon = new WPI_TalonFX(id);

        newTalon.set(ControlMode.PercentOutput, 0);
        newTalon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        newTalon.clearMotionProfileHasUnderrun(TIMEOUT_MS);

        newTalon.clearStickyFaults(TIMEOUT_MS);

        newTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.Disabled, TIMEOUT_MS);
        newTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.Disabled, TIMEOUT_MS);
        newTalon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

        // Turn off re-zeroing by default.
        newTalon.configSetParameter(
                ParamEnum.eClearPositionOnLimitF, 0, 0, 0, TIMEOUT_MS);
        newTalon.configSetParameter(
                ParamEnum.eClearPositionOnLimitR, 0, 0, 0, TIMEOUT_MS);

        newTalon.configNominalOutputForward(0, TIMEOUT_MS);
        newTalon.configNominalOutputReverse(0, TIMEOUT_MS);
        newTalon.configNeutralDeadband(config.NEUTRAL_DEADBAND, TIMEOUT_MS);

        newTalon.configPeakOutputForward(1.0, TIMEOUT_MS);
        newTalon.configPeakOutputReverse(-1.0, TIMEOUT_MS);

        newTalon.setNeutralMode(config.NEUTRAL_MODE);

        newTalon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, TIMEOUT_MS);
        newTalon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, TIMEOUT_MS);

        newTalon.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, TIMEOUT_MS);
        newTalon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, TIMEOUT_MS);
        newTalon.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

        newTalon.setInverted(config.INVERTED);
        newTalon.setSensorPhase(config.SENSOR_PHASE);

        newTalon.selectProfileSlot(0, 0);

        newTalon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, TIMEOUT_MS);
        newTalon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW,
                TIMEOUT_MS);

        newTalon.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, TIMEOUT_MS);
        newTalon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, TIMEOUT_MS);

        newTalon.configVoltageCompSaturation(0.0, TIMEOUT_MS);
        newTalon.configVoltageMeasurementFilter(32, TIMEOUT_MS);
        newTalon.enableVoltageCompensation(false);

        newTalon.configStatorCurrentLimit(config.STATOR_CURRENT_LIMIT);
        newTalon.configSupplyCurrentLimit(config.SUPPLY_CURRENT_LIMIT);

        newTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,
                config.GENERAL_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
        newTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                config.FEEDBACK_STATUS_FRAME_RATE_MS, TIMEOUT_MS);

        newTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,
                config.QUAD_ENCODER_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
        newTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
                config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
        newTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
                config.PULSE_WIDTH_STATUS_FRAME_RATE_MS, TIMEOUT_MS);

        newTalon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

        return newTalon;
    }
}
