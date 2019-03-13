package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.CentPot;
import frc.robot.util.Vector;
import frc.robot.commands.Drive;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;

// Limit switch ports: 9, 8, 7 for top, low, lowest

// The drivetrain itself
public class DriveSystem extends Subsystem {
    public static final double MOVE_MENT = .2;

    public static final double LENGTH = 23, WIDTH = 22;

    private static final double TURN_ANGLE = Math.toDegrees(Math.atan2(LENGTH, WIDTH)); // The reference angle each wheel's turning angle is based on

    private static final double[][] RELATIVE_ANGLES = {{TURN_ANGLE, 90 + TURN_ANGLE}, // The angle of each wheel when turning
        {-TURN_ANGLE, TURN_ANGLE - 180}};

    public static final double[][] P = { { .03, .04 }, { .04, .035 } }; // P Constant for each wheel

    public static final double[][] I = { { .001, .001 }, { .001, .001 } }; // I Constant for each wheel

    public static final double[][] D = { { .02, .02 }, { .02, .02 } }; // D Constnat for each wheel

    public static final int[][] DRIVE_PORTS = { { 4, 6 }, { 2, 0 } }; // Ports for the driving motors

    public static final int[][] TURN_PORTS = { { 5, 7 }, { 3, 1 } }; // Ports for the turning motors

    private Spark[][] driveSparks = { { new Spark(DRIVE_PORTS[0][0]), new Spark(DRIVE_PORTS[0][1]) },
            { new Spark(DRIVE_PORTS[1][0]), new Spark(DRIVE_PORTS[1][1]) } };

    private Spark[][] turnSparks = { { new Spark(TURN_PORTS[0][0]), new Spark(TURN_PORTS[0][1]) },
            { new Spark(TURN_PORTS[1][0]), new Spark(TURN_PORTS[1][1]) } };

    private static final int[][] ENC_PORTS = { { 1, 0 }, { 3, 2 } }; // Ports of the encoders

    private static final double[][] AVG_OFF = { { -.5, -0.02606189311184315},
                                                { 1.003883850457414, -0.7130228173596729} };

    public double lastAngle = 0; // The last angle considered 'intentional'

    public CentPot[][] encoders = {{ new CentPot(ENC_PORTS[0][0], 360, 0, AVG_OFF[0][0]), 
                                        new CentPot(ENC_PORTS[0][1], 360, 0, AVG_OFF[0][1])}, 
                                   { new CentPot(ENC_PORTS[1][0], 360, 0, AVG_OFF[1][0]), 
                                        new CentPot(ENC_PORTS[1][1], 360, 0, AVG_OFF[1][1])}};

    public PIDController[][] pids = { { new PIDController(P[0][0], I[0][0], D[0][0], encoders[0][0], turnSparks[0][0]),
            new PIDController(P[0][1], I[0][1], D[0][1], encoders[0][1], turnSparks[0][1]) },
            { new PIDController(P[1][0], I[1][0], D[1][0], encoders[1][0], turnSparks[1][0]),
                    new PIDController(P[1][1], I[1][1], D[1][1], encoders[1][1], turnSparks[1][1]) } };


    public DriveSystem()
    {
        // Set all PIDS with:
        for (PIDController[] side : pids)
            for (PIDController pid : side)
            {
                pid.setInputRange(-180, 180); // A domain of [-180, 180]
                pid.setOutputRange(-1, 1); // A range of [-1, 1]
                pid.setContinuous(); // A continuous input
            }

        // Set all sparks with:
        for (Spark[] side : turnSparks)
            for (Spark turnSpark : side)
                turnSpark.setInverted(true); // An inverted axis

        driveSparks[0][0].setInverted(true);
        driveSparks[1][0].setInverted(true); // Fix problem in random drive motor

        lastAngle = 0;
    }

    // Return the rotational vectors to turn at a given magnitude
    public static Vector[][] getRots(double mag)
    {
        Vector[][] out = new Vector[2][2];
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
            {
                out[i][j] = new Vector();
                out[i][j].setPol(mag, RELATIVE_ANGLES[i][j]);
            }
        return out;
    }

    public void enable()
    {
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                pids[i][j].enable();
    }

    // Set the command which runs when no others require the DriveSystem
    @Override
    public void initDefaultCommand()
    {
        setDefaultCommand(new Drive()); // Use the drive command if no others exist
    }

    public void setVects(Vector[][] vects)
    {
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
            {
                Vector vect = vects[i][j];

                if (vect.getMag() > MOVE_MENT)
                {
                    double err = vect.getAngle() - encoders[i][j].get();

                    if (Math.abs(err) > 90)
                        vect.negate();

                    pids[i][j].setSetpoint(vect.getAngle());

                    SmartDashboard.putNumber("targ"+i+j, vect.getAngle());
                    SmartDashboard.putNumber("curr"+i+j, encoders[i][j].get());
                    SmartDashboard.putNumber("err"+i+j, pids[i][j].getError());

                    driveSparks[i][j].set(Math.cos(Math.toRadians(err)) * vect.getMag());
                }
                else
                    driveSparks[i][j].set(0);
            }
    }

    // Return the error of a given PID
    public double getError(int i, int j) 
    {
        return pids[i][j].getError();
    }

    // Stop all motors
    public void stop()
    {
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                driveSparks[i][j].set(0);
    }

    // Totally disable the drivetrain
    public void disable()
    {
        for (int i = 0; i < 2; i++) // For each position:
        {
            for (int j = 0; j < 2; j++)
            {
                pids[i][j].disable(); // Disable the PID controller
                turnSparks[i][j].disable(); // Disable the turn motors
                driveSparks[i][j].disable(); // Disable the power motors
            }
        }
    }

    // Dealocate
    public void free()
    {
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
            {
                pids[i][j].free();
                encoders[i][j].free();
            }
    }
}