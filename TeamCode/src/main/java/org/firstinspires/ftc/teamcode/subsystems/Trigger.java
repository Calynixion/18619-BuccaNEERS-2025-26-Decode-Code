package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Trigger extends SubsystemBase {

    CRServo ServoTi;

    Telemetry telemetry;
    double power=1;
    public Trigger(HardwareMap hardwareMap, Telemetry telemetry){
        ServoTi = new CRServo(hardwareMap, "trigger");
        this.telemetry=telemetry;
    }

    public void shoot(){
        ServoTi.set(power);
    }
    public void stop(){
        ServoTi.set(0);
    }
}
