// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
// /* Open Source Software - may be modified and shared by FRC teams. The code     */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                                                                                             */
// /*----------------------------------------------------------------------------*/

// package frc.robot.subsystems;

// import frc.robot.commands.Hatch;

// import edu.wpi.first.wpilibj.command.PIDSubsystem;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.ControlMode;
    

// public class HatchSystem extends PIDSubsystem {

//     private static final double P = .6;
//     private static final double I = .005;
//     private static final double D = .005;

//     private static final int M_PORT = 2;
//     private static final int[][] SOL_PORTS = {{0, 1},{2, 3}};
    
//     public static final double MIN = -2200, MAX = 3160;
//     public static final double MAX_VOLTAGE = .4;

//     private TalonSRX motor = new TalonSRX(M_PORT);

//     private DoubleSolenoid openSol = new DoubleSolenoid(SOL_PORTS[0][0], SOL_PORTS[0][1]);
    
//     private DoubleSolenoid tiltSol = new DoubleSolenoid(SOL_PORTS[1][0], SOL_PORTS[1][1]);
    
//     public HatchSystem() 
//     {
//         super(P, I, D);
//         // setInputRange(0, MAX_ANGLE);
//         setOutputRange(-1, 1);
//         setInputRange(-1, 1);

//         setPercentTolerance(10);
//     }

//     @Override
//     public void initDefaultCommand() 
//     {
//         setDefaultCommand(new Hatch());
//     }

//     @Override
//     protected double returnPIDInput() 
//     {
//         double out = motor.getSensorCollection().getPulseWidthPosition();
//         return (out - ((MIN+MAX)/2)) / ((MAX-MIN)/2);
//     }

//     @Override
//     protected void usePIDOutput(double output) 
//     {
//         SmartDashboard.putNumber("PID set", getSetpoint());
//         SmartDashboard.putNumber("PID in", returnPIDInput());
//         SmartDashboard.putNumber("PID error", getPIDController().getError());
//         SmartDashboard.putNumber("PID out", output);

//         if (output > MAX_VOLTAGE)
//             output = MAX_VOLTAGE;
//         else if (output < -MAX_VOLTAGE)
//             output = -MAX_VOLTAGE;

//         motor.set(ControlMode.PercentOutput, output);
//     }

//     public void moveInc(double input)
//     {
//         moveTo(getPIDController().getSetpoint() + input);
//     }

//     public void moveTo(double input)
//     {
//         setSetpoint(input);
//     }

//     public void toggleHatch()
//     {
//         openSol.set(openSol.get() == Value.kForward ? Value.kReverse : Value.kForward);
//     }

//     public void toggleTilt()
//     {
//         tiltSol.set(tiltSol.get() == Value.kForward ? Value.kReverse : Value.kForward);
//     }

//     public void disable()
//     {
//         super.disable();
//         openSol.close();
//         tiltSol.close();
//         // motor.set(ControlMode.PercentOutput, 0);
//     }

//     public void free()
//     {
//         super.free();
//         motor.DestroyObject();
//         openSol.free();
//         tiltSol.free();
//     }

//     public double getAngle()
//     {
//         return motor.getSensorCollection().getPulseWidthPosition();
//     }
// }