package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.util.Vector;

public class IO
{
    public static Joystick mainJ = new Joystick(0);
    public static XboxController mainX = new XboxController(1);

    public static double z()
    {
        return mainJ.getZ() + mainX.getX(Hand.kRight);
    }

    public static double x()
    {
        return mainJ.getX() + mainX.getX(Hand.kLeft);
    }

    public static double y()
    {
        return -mainJ.getY() - mainX.getY(Hand.kLeft);
    }

    public static double transDirection()
    {
        Vector v = new Vector(IO.x(), IO.y());
        return v.getAngle();
    }

    public static double transMag()
    {
        Vector v = new Vector(IO.x(), IO.y());
        v.scale(1 / Math.sqrt(2));
        return v.getMag();
    }

    public static boolean getReset()
    {
        return mainJ.getRawButtonPressed(12) || mainX.getStartButtonPressed();
    }

    public static Joystick getJoy()
    {
        return mainJ;
    }

    public static XboxController getXbox()
    {
        return mainX;
    }
}