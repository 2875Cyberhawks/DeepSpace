package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.util.Vector;

public class IO
{
    private static final double TURN_MULT = .6;

    public static Joystick mainJ = new Joystick(0);

    public static XboxController second = new XboxController(1);

    private static final double IN_DEAD = .2;
    private static final double TRN_DEAD = .25; // Account for slight sticking on Joystick

    private static final double MAX_CHANGE = .085;
    private static double prevZ = 0, prevT = 0;
    private static boolean applyCurve = true;
    private static final double CREEP_SPEED = .4;

    private static final double HATCH_SCALE = 1;
    private static final double BALL_SCALE = 1;
    private static final double INTAKE_SCALE = .8;
    private static final double CLIMB_WHEEL_SPEED = .6;

    public static double z()
    {
        double in = TURN_MULT * mainJ.getZ();

        if (Math.abs(in) < TRN_DEAD)
            in = 0;

        if (mainJ.getRawButton(5))        
            in *= CREEP_SPEED;

        if (!applyCurve)
            return in;

        double change = -prevZ + in;
        if (Math.abs(change) > MAX_CHANGE)
            change = MAX_CHANGE * (change > 0 ? 1 : -1);
        double fin = prevZ + change;
        prevZ = fin;

        return fin;
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

        if (Math.abs(v.getMag()) < IN_DEAD)
            v.scale(.00001); // Negligable - Doesn't move wheels but keeps position

        if (mainJ.getRawButton(5))        
            v.scale(CREEP_SPEED);

        if (!applyCurve)
            return v;

        double change = -prevT + v.getMag();
        if (Math.abs(change) > MAX_CHANGE)
            change = MAX_CHANGE * (change > 0 ? 1 : -1);
        double fin = prevT + change;
        prevT = fin;
        v.setMag(fin);

        return v;
    }

    public static boolean getReset()
    {
        return mainJ.getRawButtonPressed(8);
    }

    public static Joystick getJoy()
    {
        return mainJ;
    }

    public static XboxController getXbox()
    {
        return second;
    }

    public static double lift()
    {
        if (second.getPOV() == 0)
            return 1;
        else if (second.getPOV() == 180)
            return -1;
        else
            return 0;
    }

    public static double hatchAxis()
    {
        return Math.abs(second.getY(Hand.kLeft)) > IN_DEAD ? second.getY(Hand.kLeft) * HATCH_SCALE : 0;
    }

    public static boolean toggleHatch()
    {
        return  second.getYButtonPressed();
    }

    public static boolean toggleTilt()
    {
        return second.getXButtonPressed();
    }

    public static double ballAxis()
    {
        return Math.abs(second.getY(Hand.kRight)) > IN_DEAD ? -second.getY(Hand.kRight) * BALL_SCALE : 0;
    }

    public static double intakeBall()
    {
        return second.getTriggerAxis(Hand.kLeft) > IN_DEAD ? second.getTriggerAxis(Hand.kLeft) * INTAKE_SCALE : 0;
    }

    public static double shootBall()
    {
        return second.getTriggerAxis(Hand.kRight) > IN_DEAD ? second.getTriggerAxis(Hand.kRight) : 0;
    }

    public static double climbWheels()
    {
        if (mainJ.getRawButtonPressed(11))
            return CLIMB_WHEEL_SPEED;
        else if (mainJ.getRawButtonPressed(12))
            return -CLIMB_WHEEL_SPEED;
        else
            return 0;
    }

    public static boolean toggleClimb()
    {
        return mainJ.getRawButtonPressed(9);
    }

    public static boolean switchCamera()
    {
        return mainJ.getRawButtonPressed(10);
    }

    public static boolean getCenterHatch()
    {
        return second.getBButtonPressed();
    }

    public static boolean getCenterBall()
    {
        return second.getAButtonPressed();
    }

    public static boolean manualClimb()
    {
        return second.getStartButtonPressed();
    }
}