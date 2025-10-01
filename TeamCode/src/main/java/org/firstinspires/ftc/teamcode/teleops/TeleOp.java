package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commands.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

//OpMode is actual code that is initialized and ran, calls commands which call methods of subsystems
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="DefaultTeleOp")
public class TeleOp extends CommandOpMode {
    //initialize variables
    Drive drivetrain;
    RobotCentricDrive r_drive;
    GamepadEx controller1;

    //initialize function runs when init is pressed on the driver station with this teleop selected
    @Override
    public void initialize(){
        //assign variables to their respective objects
        controller1 = new GamepadEx(gamepad1);
        drivetrain = new Drive(hardwareMap, telemetry);
        r_drive = new RobotCentricDrive(drivetrain,controller1::getLeftX,controller1::getLeftY,controller1::getRightX);

        //sets the drivetrain subsystem to run the robot centric command continuously
        drivetrain.setDefaultCommand(r_drive);
    }


}
