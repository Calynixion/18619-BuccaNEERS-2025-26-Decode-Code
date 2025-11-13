package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SubsystemBase {

    Motor IntakeM;
    Telemetry telemetry;
    double power = -1;
    public Intake(HardwareMap hwMap, Telemetry telemetry){
        this.IntakeM = new Motor(hwMap,"IntakeM");
        this.telemetry = telemetry;

        IntakeM.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void spin(){
        IntakeM.set(power);
    }
    public void reverseSpin(){
        IntakeM.set(-power);}
    public void stop(){
        IntakeM.set(0);
    }



}
