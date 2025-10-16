package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.commands.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

//OpMode is actual code that is initialized and ran, calls commands which call methods of subsystems
@TeleOp(name="DefaultTeleOp")
public class DefaultTeleOp extends CommandOpMode {
    //initialize variables
    Drive drivetrain;
    RobotCentricDrive r_drive;
    GamepadEx controller1;
    Intake intake;
    Outtake outtake;

    //initialize function runs when init is pressed on the driver station with this teleop selected
    @Override
    public void initialize(){
        //assign variables to their respective objects
        controller1 = new GamepadEx(gamepad1);
        drivetrain = new Drive(hardwareMap, telemetry);
        r_drive = new RobotCentricDrive(drivetrain,controller1::getLeftX,controller1::getLeftY,controller1::getRightX);

        intake = new Intake(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);
        register(drivetrain,intake,outtake);
        //sets the drivetrain subsystem to run the robot centric command continuously
        drivetrain.setDefaultCommand(r_drive);
        /*
        controller1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(intake::spin))
                .whenReleased(new InstantCommand(intake::stop));

        controller1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(outtake::spin))
                .whenReleased(new InstantCommand(outtake::stop));
        */



    }



}
