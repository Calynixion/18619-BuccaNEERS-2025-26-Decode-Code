package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

import java.util.function.DoubleSupplier;

//this is a command, it tells the subsystem, the representation of the physical parts, what to do
//  to accomplish a specific action
public class RobotCentricDrive extends CommandBase {
    //initialize variables
    private final Drive drivetrain;
    private final DoubleSupplier strafe, forward, turn;

    //constructor for the command, sets the given variables to local variables
    public RobotCentricDrive(Drive drivetrain, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn) {

        this.drivetrain = drivetrain;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
        addRequirements(drivetrain);
    }

    //continuously calls the movement method in the drive class
    @Override
    public void execute(){
        drivetrain.robotCentricDrive(-strafe.getAsDouble(), -forward.getAsDouble(), -turn.getAsDouble());
    }
}


