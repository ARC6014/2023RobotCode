package frc.team6014.lib.util;

public class FieldRelativeAccel {
    public double ax;
    public double ay;
    public double alpha;

    public FieldRelativeAccel(double ax, double ay, double alpha) {
        this.ax = ax;
        this.ay = ay;
        this.alpha = alpha;
    }

    public FieldRelativeAccel(FieldRelativeSpeed newSpeed, FieldRelativeSpeed oldSpeed, double time) {
        this.ax = (newSpeed.m_vx - oldSpeed.m_vx) / time;
        this.ay = (newSpeed.m_vy - oldSpeed.m_vy) / time;
        this.alpha = (newSpeed.m_omega - oldSpeed.m_omega) / time;

        if (Math.abs(this.ax) > 6.0) {
            this.ax = 6.0 * Math.signum(this.ax);
        }
        if (Math.abs(this.ay) > 6.0) {
            this.ay = 6.0 * Math.signum(this.ay);
        }
        if (Math.abs(this.alpha) > 4 * Math.PI) {
            this.alpha = 4 * Math.PI * Math.signum(this.alpha);
        }
    }

    public FieldRelativeAccel() {
        this.ax = 0.0;
        this.ay = 0.0;
        this.alpha = 0.0;
    }

}
