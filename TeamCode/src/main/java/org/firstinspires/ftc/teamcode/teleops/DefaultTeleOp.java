package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.commands.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

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

    //initialize function runs when init is pressed on the driver station with this teleop selected
    @Override
    public void initialize(){
        //assign variables to their respective objects
        controller1 = new GamepadEx(gamepad1);
        drivetrain = new Drive(hardwareMap, telemetry);
        r_drive = new RobotCentricDrive(drivetrain,controller1::getLeftX,controller1::getLeftY,controller1::getRightX);
        intake = new Intake(hardwareMap, telemetry);

        //register the subsystems to the command scheduler
        register(drivetrain,intake);

        //sets the drivetrain subsystem to run the robot centric command continuously
        drivetrain.setDefaultCommand(r_drive);

        //example of instant commands to call directly from subsystems without a custom command
        controller1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(intake::spin))
                .whenReleased(new InstantCommand(intake::stop));

    }


}
