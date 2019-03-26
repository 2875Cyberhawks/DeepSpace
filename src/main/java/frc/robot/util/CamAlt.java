package frc.robot.util;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;

// Camera Alternator
public class CamAlt
{
    private int camNum = 0; // Defaults on camera 0
    private UsbCamera[] cams;

   public CamAlt(int[] cPorts)
    {
        cams = new UsbCamera[cPorts.length];
        for (int i = 0; i < cPorts.length; i++)
        {
            cams[i] = CameraServer.getInstance().startAutomaticCapture(i);
            cams[i].setBrightness(10);
            System.out.println(cams[i].setFPS(10) ? "Cam "+i+" FPS" : "Cam "+i+" no FPS");
            System.out.println(cams[i].setResolution(160, 120) ? "Cam "+i+" res" : "Cam "+i+" no res");
        }
        setCam(cams[0]);
    }

    public void inc()
    {
        camNum = ++camNum > cams.length-1 ? 0 : camNum;
        setCam(cams[camNum]);
    }

    public void dec()
    {
        camNum = --camNum < 0 ? cams.length-1 : camNum;
        setCam(cams[camNum]);
    }

    public static void setCam(UsbCamera c)
    {
        NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection").setString(c.getName());
    }
}