package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

@Autonomous(name="RedAuto")
public class RedAuto extends LinearOpMode {
    Drive drivetrain;
    @Override
    public void runOpMode(){
        drivetrain = new Drive(hardwareMap,telemetry);
        telemetry.addData("Status","Ready to begin Auto");
        telemetry.addData("Version","AfterSlothLoad");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            drivetrain.robotCentricDrive(0, -1, 0);
        }
    }
}
