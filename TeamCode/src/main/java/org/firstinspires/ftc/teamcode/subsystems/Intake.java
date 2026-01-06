package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SubsystemBase {

    MotorEx IntakeM;

    MotorGroup intakeTransfer;
    Telemetry telemetry;
    double power = 1;
    public Intake(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.IntakeM = new MotorEx(hwMap,"IntakeM");
        intakeTransfer = new MotorGroup(IntakeM);
        intakeTransfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void spin(){
        intakeTransfer.set(-power);
    }
    public void reverseSpin(){
        intakeTransfer.set(power);
    }
    public void stop(){
        intakeTransfer.set(0);
    }



}
