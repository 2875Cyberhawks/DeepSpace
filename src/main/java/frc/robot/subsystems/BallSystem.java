package frc.robot.subsystems;

import frc.robot.commands.Ball;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class BallSystem extends Subsystem {

    private static final double P = 2;
    private static final double I = .001;
    private static final double D = 0;

    private static final int[] DEVICE_NUMS = {1, 12}; // Turn, Lower
    public static final double FULL_TURN = 4096; // One full turn of the encoder

    public static final double MIN = -3250, MAX = 800; // Forward, BACK
    public static final double TO_MAX = 100, TO_MIN = 100; // Distance from the max and min to the center
    private double center = 600; // VALUE TO FIND
    public TalonSRX rotTal = new TalonSRX(DEVICE_NUMS[0]);
    public boolean limited = true;
    public double setpoint = 0;

    private Talon shoot = new Talon(DEVICE_NUMS[1]);

    public BallSystem() 
    {
        shoot.set(0);

        rotTal.configFactoryDefault();

        rotTal.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rotTal.setSelectedSensorPosition(0);
        
        rotTal.configMotionCruiseVelocity(750);
        rotTal.configMotionAcceleration(1500);
    
        rotTal.configContinuousCurrentLimit(10);
        rotTal.enableCurrentLimit(true);
    
        rotTal.configNominalOutputForward(0);
        rotTal.configNominalOutputReverse(0);
        rotTal.configPeakOutputForward(1);
        rotTal.configPeakOutputReverse(-1);
        rotTal.setIntegralAccumulator(0);
        rotTal.config_IntegralZone(0, 75);

        rotTal.config_kP(0, P);
        rotTal.config_kI(0, I); 
        rotTal.config_kD(0, D);
        rotTal.config_kF(0, 0);
    }

    @Override
    public void initDefaultCommand() 
    {
        setDefaultCommand(new Ball());    
    }

    public void moveInc(double diff) // Send in a positional difference in
    {
        if (limited)
            if (getAbs() > (center + TO_MAX) && diff > 0)
                diff = 0;
            else if (getAbs() < (center - TO_MIN) && diff < 0)
                diff = 0;
        
        moveTo(setpoint + diff);
    }

    public void moveTo(double input)
    {
        //rotTal.set(ControlMode.MotionMagic, input);
        setpoint = input;
    }

    public double setCent()
    {
        return center = rotTal.getSelectedSensorPosition();
    }

    public void shoot(double speed)
    {
        shoot.set(speed);
    }

    public void disable()
    {
        rotTal.set(ControlMode.PercentOutput, 0);
        shoot.set(0);
    }

    public double getAbs()
    {
        return rotTal.getSensorCollection().getPulseWidthPosition();
    }
}