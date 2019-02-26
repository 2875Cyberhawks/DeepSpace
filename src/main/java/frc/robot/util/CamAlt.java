package frc.robot.util;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;

// Camera Alternator
public class CamAlt
{
    boolean onCam0 = true; // Defaults on camera 0
    UsbCamera cam0, cam1;
    int[] ports = new int[2];

    public CamAlt(int c0Port, int c1Port)
    {
        ports[0] = c0Port;
        ports[1] = c1Port;
    }

    public void init()
    {
        cam0 = CameraServer.getInstance().startAutomaticCapture(ports[0]);
        cam1 = CameraServer.getInstance().startAutomaticCapture(ports[1]);

        setCam(cam0);
    }

    public void toggle()
    {
        onCam0 = !onCam0;

        setCam(onCam0 ? cam0 : cam1);
    }

    public static void setCam(UsbCamera c)
    {
        NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection").setString(c.getName());
    }
}