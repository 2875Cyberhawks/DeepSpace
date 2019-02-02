package frc.robot.subsystems;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.util.CentPot;
import frc.robot.commands.Drive;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;

// The drivetrain itself
public class DriveSystem extends Subsystem {
    public static final double[][] P = { { .03, .04 }, { .04, .035 } }; // P Constant for each wheel

    public static final double[][] I = { { .001, .001 }, { .001, .001 } }; // I Constant for each wheel

    public static final double[][] D = { { .02, .02 }, { .02, .02 } }; // D Constnat for each wheel

    public static final int[][] DRIVE_PORTS = { { 0, 2 }, { 6, 4 } }; // Ports for the driving motors

    public static final int[][] TURN_PORTS = { { 7, 1 }, { 5, 3 } }; // Ports for the turning motors

    private Spark[][] driveSparks = { { new Spark(DRIVE_PORTS[0][0]), new Spark(DRIVE_PORTS[0][1]) },
            { new Spark(DRIVE_PORTS[1][0]), new Spark(DRIVE_PORTS[1][1]) } };

    private Spark[][] turnSparks = { { new Spark(TURN_PORTS[0][0]), new Spark(TURN_PORTS[0][1]) },
            { new Spark(TURN_PORTS[1][0]), new Spark(TURN_PORTS[1][1]) } };

    private static final int[][] ENC_PORTS = { { 2, 0 }, { 3, 1 } }; // Ports of the encoders

    private static final double[][] AVG_OFF = { { 0.2815926584972559, -0.332589036651513 },
                                                { 0.03405286522612938, -0.01819194111990008 } }; 

    // public AnalogPotentiometer[][] encoders = { { new AnalogPotentiometer(ENC_PORTS[0][0], -1, 0),
    //         new AnalogPotentiometer(ENC_PORTS[0][1], -1, 0) },
    //         { new AnalogPotentiometer(ENC_PORTS[1][0], -1, 0),
    //                 new AnalogPotentiometer(ENC_PORTS[1][1], -1, 0) } };

    public double lastAngle = 0; // The last angle considered 'intentional'
    public boolean turnMeant = false;

    private static final double STAT_DEAD = 5;// The maximum total speed at which the robot is considered stationary

    public CentPot[][] encoders = {{ new CentPot(ENC_PORTS[0][0], 360, 0, AVG_OFF[0][0]), 
                                        new CentPot(ENC_PORTS[0][1], 360, 0, AVG_OFF[0][1])}, 
                                   { new CentPot(ENC_PORTS[1][0], 360, 0, AVG_OFF[1][0]), 
                                        new CentPot(ENC_PORTS[1][1], 360, 0, AVG_OFF[1][0])}};

    private PIDController[][] pids = { { new PIDController(P[0][0], I[0][0], D[0][0], encoders[0][0], turnSparks[0][0]),
            new PIDController(P[0][1], I[0][1], D[0][1], encoders[0][1], turnSparks[0][1]) },
            { new PIDController(P[1][0], I[1][0], D[1][0], encoders[1][0], turnSparks[1][0]),
                    new PIDController(P[1][1], I[1][1], D[1][1], encoders[1][1], turnSparks[1][1]) } };

    public DriveSystem()
    {
        // Set all PIDS with:
        for (PIDController[] side : pids)
        {
            for (PIDController pid : side)
            {
                pid.setInputRange(-180, 180); // A domain of [-180, 180]
                pid.setOutputRange(-1, 1); // A range of [-1, 1]
                pid.setContinuous(); // A continuous input
                pid.enable(); // An enabled state
            }
        }

        // Set all sparks with:
        for (Spark[] side : turnSparks)
        {
            for (Spark turnSpark : side)
            {
                turnSpark.setInverted(true); // An inverted axis
            }
        }

        driveSparks[1][0].setInverted(true); // Fix problem in random drive motor
    }

    // Set the command which runs when no others require the DriveSystem
    @Override
    public void initDefaultCommand()
    {
        setDefaultCommand(new Drive()); // Use the drive command if no others exist
    }

    // Set the angle of a motor at position (i, j), where top-left is (0, 0) and bottom-right is (1, 1)
    public void setTurn(int i, int j, double angle)
    {
        // System.out.println("Sending angle "+angle+" to motor " + i + "-" + j);
        pids[i][j].setSetpoint(angle);
    }

    // Set the power of a motor at position (i, j), where top-left is (0, 0) and bottom-right is (1, 1)
    public void setPower(int i, int j, double power)
    {
        driveSparks[i][j].set(power);
    }

    // Return the error of a given PID
    public double getError(int i, int j) 
    {
        return pids[i][j].getError();
    }

    public boolean isTurning()
    {    
        return Math.abs(Robot.gyro.getRate()) > STAT_DEAD;
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
}
