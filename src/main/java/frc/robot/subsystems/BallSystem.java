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

    private static final double P = 0;
    private static final double I = 0;
    private static final double D = 0;

    private static final int[] DEVICE_NUMS = {1, 12}; // Turn, Lower
    public static final double FULL_TURN = 4096; // One full turn of the encoder

    public static final double MIN = -3045, MAX = 600; // Forward, Straight, Back
    private TalonSRX rotTal = new TalonSRX(DEVICE_NUMS[0]);

    private double setpoint = 0;

    private static final double MAX_VOLTAGE = .4;

    private static final double MAX_TURN_SPEED = 1;

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
        if (getAbs() > MAX && diff > 0)
            diff = 0;
        else if (getAbs() < MIN && diff < 0)
            diff = 0;
        moveTo(setpoint + diff);
    }

    public void moveTo(double input)
    {
        rotTal.set(ControlMode.MotionMagic, input);
        setpoint = input;
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