package frc.team6014.lib.math;

public class Gearbox {

    private double m_gearRatio;

    public Gearbox(double drivingGear, double drivenGear) {
        m_gearRatio = drivenGear / drivingGear;
    }

    public Gearbox(double gearRatio) {
        m_gearRatio = gearRatio;
    }

    public double calculate(double rot) {
        return rot * m_gearRatio;
    }

    public double getRatio(){
        return m_gearRatio;
    }
}
