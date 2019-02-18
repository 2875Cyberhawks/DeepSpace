package frc.robot;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private static final double HATCH_SCALE = .3;
    private static final double BALL_SCALE = .5;
    private static final double INTAKE_SCALE = .4;
    private static final double CLIMB_WHEEL_SPEED = .6;
    private static final double LIFT_SPEED = .5;

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
        return mainJ.getRawButtonPressed(7);
    }

    public static Joystick getJoy()
    {
        return mainJ;
    }

    public static XboxController getXbox()
    {
        return second;
    }

    /*public static boolean lowLift(){
        return second.getPOV() == 270;
    }

    public static boolean midLift(){
        return second.getPOV() == 0 ;
    }

    public static boolean highLift(){
        return second.getPOV() == 90;
    } */

    public static double lift()
    {
        if (second.getPOV() == 0)
            return 1;
        else if (second.getPOV() == 180)
            return -1;
        else
            return 0;

        // if (backup.getRawButton(3))
        //     return 1;
        // else if (backup.getRawButton(2))
        //     return -1;
        // else 
        //     return 0;
    }

    public static double hatchAxis(){
        return Math.abs(second.getY(Hand.kLeft)) > IN_DEAD ? -second.getY(Hand.kLeft) * HATCH_SCALE : 0;
    }

    public static boolean toggleHatch(){
        return  second.getYButtonPressed();
    }

    public static boolean toggleTilt(){
        return second.getXButtonPressed();
    }

    public static double ballAxis(){
        return Math.abs(second.getY(Hand.kRight)) > IN_DEAD ? -second.getY(Hand.kRight) * BALL_SCALE : 0;
    }

    public static double intakeBall(){
        return second.getTriggerAxis(Hand.kRight) > IN_DEAD ? second.getTriggerAxis(Hand.kRight) * INTAKE_SCALE : 0;
    }

    public static double shootBall(){
        return second.getTriggerAxis(Hand.kLeft) > IN_DEAD ? second.getTriggerAxis(Hand.kLeft) : 0;
    }

    public static double climbWheels(){
        if (mainJ.getRawButtonPressed(11))
            return CLIMB_WHEEL_SPEED;
        else if (mainJ.getRawButtonPressed(12))
            return -CLIMB_WHEEL_SPEED;
        else
            return 0;
    }

    public static boolean toggleClimb(){
        return mainJ.getRawButtonPressed(9);
    }
}