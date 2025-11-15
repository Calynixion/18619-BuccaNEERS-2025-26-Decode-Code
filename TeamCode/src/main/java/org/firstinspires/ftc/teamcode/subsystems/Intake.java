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

    MotorEx IntakeM2;

    MotorGroup intakeTransfer;
    Telemetry telemetry;
    double power = -1;
    public Intake(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.IntakeM = new MotorEx(hwMap,"IntakeM");
        this.IntakeM2 = new MotorEx(hwMap, "IntakeM2");
        IntakeM2.setInverted(true);
        intakeTransfer = new MotorGroup(IntakeM,IntakeM2);
        intakeTransfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void spin(){
        intakeTransfer.set(power);
    }
    public void reverseSpin(){
        intakeTransfer.set(-power);
    }
    public void stop(){
        intakeTransfer.set(0);
    }



}
