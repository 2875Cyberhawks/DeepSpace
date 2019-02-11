package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.util.Vector;

public class IO
{
    private static final double TURN_MULT = .75;

    public static Joystick mainJ = new Joystick(0);

    public static XboxController second = new XboxController(1);

    private static final double IN_DEAD = .2;

    private static final double MAX_CHANGE = .085;
    private static double prevZ = 0, prevT = 0;
    private static boolean applyCurve = true;

    private static final double LOW = 1;
    private static final double MID = 2;
    private static final double HIGH = 3;

    public static double z()
    {
        double in = TURN_MULT * mainJ.getZ();

        if (!applyCurve)
            return in;

        double change = -prevZ + in;
        if (Math.abs(change) > MAX_CHANGE)
            change = MAX_CHANGE * (change > 0 ? 1 : -1);
        double fin = prevZ + change;
        prevZ = fin;

        if (Math.abs(fin) > IN_DEAD)
            return fin;
        else
            return 0;
    }

    private static double rawX()
    {
         return mainJ.getX();
    }

    private static double rawY()
    {
        return -mainJ.getY();
    }

    public static Vector trans()
    {
        Vector v = new Vector(IO.rawX(), IO.rawY());
        
        if (!applyCurve)
            return v;

        double change = -prevT + v.getMag();
        if (Math.abs(change) > MAX_CHANGE)
            change = MAX_CHANGE * (change > 0 ? 1 : -1);
        double fin = prevT + change;
        prevT = fin;
        v.scale(v.getMag() / fin);

        if (v.getMag() > IN_DEAD)
            return v;
        else
            return new Vector();
    }

    public static boolean getReset()
    {
        return mainJ.getRawButtonPressed(12);
    }

    public static Joystick getJoy()
    {
        return mainJ;
    }

    public static XboxController getXbox()
    {
        return second;
    }

    public static double lowLift(){
        return ()
    }
}