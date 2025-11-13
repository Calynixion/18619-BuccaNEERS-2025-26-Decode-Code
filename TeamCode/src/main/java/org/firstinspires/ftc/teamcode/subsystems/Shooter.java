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

public class Shooter extends SubsystemBase {

    Motor ShooterA;
    Motor ShooterB;
    Telemetry telemetry;
    double power = 1;
    public Shooter(HardwareMap hwMap, Telemetry telemetry){
        this.ShooterA = new Motor(hwMap,"ShooterA");
        this.ShooterB = new Motor(hwMap, "ShooterB");

        this.telemetry = telemetry;
        ShooterA.setRunMode(Motor.RunMode.RawPower);
        ShooterB.setRunMode(Motor.RunMode.RawPower);
        ShooterA.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        ShooterB.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void spin(){
        ShooterA.set(power);
        ShooterB.set(-power);
    }
    public void reverseSpin(){
        ShooterA.set(-power*0.25);
        ShooterB.set(power*0.25);

    }
    public void stop(){
        ShooterA.set(0);
        ShooterB.set(0);
    }



}
