package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SubsystemBase {

    Motor IntakeM;
    Telemetry telemetry;
    double power = 1;
    public Intake(HardwareMap hwMap, Telemetry telemetry){
        this.IntakeM = new Motor(hwMap,"IntakeM");
        this.telemetry = telemetry;

        IntakeM.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void spin(){IntakeM.set(-power);
    }
    public void stop(){
        IntakeM.set(0);
    }



}
