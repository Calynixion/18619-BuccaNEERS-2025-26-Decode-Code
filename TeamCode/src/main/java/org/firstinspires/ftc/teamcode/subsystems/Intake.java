package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SubsystemBase {
    Motor IntakeS;
    Telemetry telemetry;
    double power = 1;
    public Intake(HardwareMap hwMap, Telemetry telemetry){
        this.IntakeS = new CRServo(hwMap,"IntakeS");
        this.telemetry = telemetry;

        //IntakeS.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void spin(){IntakeS.set(power);
    }
    public void stop(){
        IntakeS.set(0);
    }

}
