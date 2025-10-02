package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.commands.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

//OpMode is actual code that is initialized and ran, calls commands which call methods of subsystems
@TeleOp(name="FieldTeleOp")
public class FieldTeleOp extends CommandOpMode {
    //initialize variables
    Drive drivetrain;
    FieldCentricDrive f_drive;
    GamepadEx controller1;

    //initialize function runs when init is pressed on the driver station with this teleop selected
    @Override
    public void initialize(){
        //assign variables to their respective objects
        controller1 = new GamepadEx(gamepad1);
        drivetrain = new Drive(hardwareMap, telemetry);
        f_drive = new FieldCentricDrive(drivetrain,controller1::getLeftX,controller1::getLeftY,controller1::getRightX);

        //sets the drivetrain subsystem to run the robot centric command continuously
        drivetrain.setDefaultCommand(f_drive);
    }
}
