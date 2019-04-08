package frc.robot.subsystems;

import frc.robot.commands.Lift;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

// The physical lift itself
public class LiftSystem extends PIDSubsystem 
{
    // The P, I and D constants
    private static final double P = .1;// .65;
    private static final double I = 0;// .0065;
    private static final double D = 0;

    // The maximum speed
    private static final double MAX_SPD = .7;

        // The ports for the Spark motorcontrollers
    private static final int MOTOR_PORT = 9;

    // The ports for the encoder
    private static final int[] ENC_PORTS = {2, 3};

    // The ports for the limit switches
    private static final int[] LIMIT_PORTS = {7, 8, 9}; // min, rest, max

    private Spark motor;

    // The encoder for tracking the height of the lift
    private Encoder encoder;

    // The minimum height the lift should reach
    private static final double MIN_HEIGHT = -14;

    // The maximum height the lift should reach
    private static final double MAX_HEIGHT = 65;

    // Constant of proportionality between the number of encoder pulses and inches
    private static final double DISTANCE_PER_PULSE = (12.0 / 577.0);

    public boolean climbMode = false;
    public DigitalInput min, rest, max;

    private boolean inRest;

    public boolean isInit;
    public Timer initTime;
    public static final double MAX_TIME_INIT = 4;
    public static final double JUMP_DISTANCE = 6;

    private boolean startedPass = false;

    public LiftSystem()
    {
        // Tell the superclass the PID values
        super(P, I, D);

        inRest = false;

        // The input should only be on this range
        setInputRange(MIN_HEIGHT, MAX_HEIGHT);

        // The output should only be on this range 
        setOutputRange(-1, 1);

        encoder = new Encoder(ENC_PORTS[0], ENC_PORTS[1]); // Create the object
        encoder.setDistancePerPulse(DISTANCE_PER_PULSE); // Set distance per pulse 

        // Create the SpeedControllerGroup
        motor = new Spark(MOTOR_PORT);

        // Create the Limit Switches
        min = new DigitalInput(LIMIT_PORTS[0]);
        rest = new DigitalInput(LIMIT_PORTS[1]);
        max = new DigitalInput(LIMIT_PORTS[2]);

        SmartDashboard.putString("Ended by", "NOT");
    }

    public void init() 
    {
        encoder.reset();
        setSetpoint(JUMP_DISTANCE);

        isInit = true;
        initTime = new Timer();
        initTime.start();
    }

    @Override
    public void initDefaultCommand() 
    {
        // Run the lift command when nothing else requires the lift
        setDefaultCommand(new Lift());
    }

    @Override
    protected double returnPIDInput() 
    {
        return getHeight();
    }

    @Override
    protected void usePIDOutput(double output) 
    {  
        SmartDashboard.putBoolean("Hatch Position", !rest.get());
        if (isInit)
        {
            // if (!startedPass && !rest.get())
            //     startedPass = true;

            if (!rest.get()/* && startedPass*/)
            {
                isInit = false;
                SmartDashboard.putString("Ended by", "SWITCH");
            }

            if (initTime.get() > MAX_TIME_INIT)
            {
                isInit = false;
                SmartDashboard.putString("Ended by", "TIME");
            }

            if (!isInit)
            {
                System.out.println("Encoder reset");
                encoder.reset();
                setSetpoint(getHeight());
                motor.set(0);
                return;
            }
        }

        SmartDashboard.putBoolean("TRIGGERING REST", false);
        if (getSetpoint() > getHeight() && !max.get())
        {
            System.out.println("Max triggered");
            setSetpoint(getHeight());
            return;
        }
        else if (getSetpoint() < getHeight() && !min.get())
        {
            System.out.println("Min triggered");
            setSetpoint(getHeight());
            return;
        }
        // else if (getSetpoint() < getHeight() && !rest.get() && !climbMode)
        // {
        //     SmartDashboard.putBoolean("TRIGGERING REST", true);
        //     // if (!inRest)
        //     //     encoder.reset();
        //     // inRest = true;
        //     // // setSetpoint(0);
        //     return;
        // }
        else
            inRest = false;

        // The output of the PID loop should be the motor's speed
        if (output > MAX_SPD)
            motor.set(MAX_SPD);
        else if (output < -MAX_SPD)
            motor.set(-MAX_SPD);
        else
            motor.set(output);
    }

    // Move to a given height (double)
    public void moveToHeight(double height)
    {
        SmartDashboard.putNumber("wantHeight", height);
        if (height > MAX_HEIGHT) // If the height is greater than the max:
        {
            // Move it to the max height
            SmartDashboard.putNumber("targHeight", MAX_HEIGHT);
            setSetpoint(MAX_HEIGHT);
        }
        else if (height < MIN_HEIGHT) // otherwise, if the height is less than the min
        {
            // Move it to the minimum
            SmartDashboard.putNumber("targHeight", MIN_HEIGHT);
            setSetpoint(MIN_HEIGHT);
        }
        else // otherwise, move it to the given height
        {    
            SmartDashboard.putNumber("targHeight", height);
            setSetpoint(height);
        }
    }

    // Getting the height is the current distance of the encoder
    public double getHeight()
    {
        return encoder.getDistance();
    }

    // Disable the motors
    public void disable()
    {
        super.disable();
        // motor.set(0);
    }

    // Free all of the HAL objects
    public void free()
    {
        super.free();
        encoder.free();
    }

    // Reset the encoder
    public void reset()
    {
        encoder.reset();
    }
}