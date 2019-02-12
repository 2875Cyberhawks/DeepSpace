/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.command.Command;

import frc.robot.subsystems.LiftSystem;
import frc.robot.IO;
import frc.robot.Robot;

public class Lift extends Command {

    private double goalHeight;

    private final double[] HEIGHTS = {0, 1, 2};

    public Lift() {
        requires(Robot.ls);
    }

    @Override
    protected void initialize() {
        Robot.ls.enable();
        goalHeight = HEIGHTS[0];
    }


    @Override
    protected void execute() {

        goalHeight = IO.lowLift() ? HEIGHTS[0] : goalHeight;
        goalHeight = IO.midLift() ? HEIGHTS[1] : goalHeight;
        goalHeight = IO.highLift() ? HEIGHTS[2] : goalHeight;

        Robot.ls.moveToHeight(goalHeight);

    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        goalHeight = HEIGHTS[0];
        Robot.ls.disable();
    }
    @Override
    protected void interrupted() {
        goalHeight = HEIGHTS[0];
        Robot.ls.disable();
    }
}
