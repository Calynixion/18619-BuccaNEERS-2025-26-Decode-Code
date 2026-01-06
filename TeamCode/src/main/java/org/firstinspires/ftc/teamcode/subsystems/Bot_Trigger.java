package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.bylazar.configurables.annotations.Configurable;
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

public class Bot_Trigger extends SubsystemBase {
    MotorEx bTrigger;
    Telemetry telemetry;
    public static double power = 1;

    public Bot_Trigger(HardwareMap hwMap, Telemetry telemetry) {
        this.bTrigger = new MotorEx(hwMap, "Trigger");
        this.telemetry = telemetry;
        bTrigger.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void shoot() {
        bTrigger.set(power);
    }

    public void reverseShoot() {
        bTrigger.set(-power);
    }

    public void stop() {
        bTrigger.set(0);
    }
    public void setPower(double power_to_set){bTrigger.set(power_to_set);}
}
/*
    CRServo ServoTi;

    Telemetry telemetry;
    double power=1;
    public Bot_Trigger(HardwareMap hardwareMap, Telemetry telemetry){
        ServoTi = new CRServo(hardwareMap, "ServoTi");
        this.telemetry=telemetry;
    }

    public void shoot(){
        ServoTi.set(power);
    }
    public void reverseShoot(){
        ServoTi.set(-1*power);
    }
    public void stop(){
        ServoTi.set(0);
    }
}
*/

