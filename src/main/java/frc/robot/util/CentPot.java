package frc.robot.util;

import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class CentPot extends AnalogPotentiometer
{
    private double start;
    private double offset;
    private double range;

    public CentPot(int port, double range, double offset, double start)
    {
        super(port, 2, -1);
        this.range = range;
        this.offset = offset;
        this.start = start;
    }

    public double pidGet()
    {
        return get();
    }

    public PIDSourceType getPIDSourceType()
    {
        return super.getPIDSourceType();
    }

    public void setPIDSourceType(PIDSourceType pidSource)
    {
        throw new UnsupportedOperationException("Does not support changing source types");
    }

    public double get()
    {
        double val = super.get() - start;
        if (val < -1)
            val += 2;
        else if (val > 1)
            val -= 2;
        val *= -(range / 2);
        return val - offset;
    }

    public double getRaw()
    {
        return super.get();
    }
}