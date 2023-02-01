package frc.team6014.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class FieldRelativeSpeed {
    public double m_vx;
    public double m_vy;
    public double m_omega;

    public FieldRelativeSpeed(double vx, double vy, double omega) {
        m_vx = vx;
        m_vy = vy;
        m_omega = omega;
    }

    public FieldRelativeSpeed(ChassisSpeeds chassisSpeed, Rotation2d gyro) {
        this(chassisSpeed.vxMetersPerSecond * gyro.getCos() - chassisSpeed.vyMetersPerSecond * gyro.getSin(),
                chassisSpeed.vyMetersPerSecond * gyro.getCos() + chassisSpeed.vxMetersPerSecond * gyro.getSin(),
                chassisSpeed.omegaRadiansPerSecond);
    }

    public FieldRelativeSpeed() {
        m_vx = 0.0;
        m_vy = 0.0;
        m_omega = 0.0;
    }

}
