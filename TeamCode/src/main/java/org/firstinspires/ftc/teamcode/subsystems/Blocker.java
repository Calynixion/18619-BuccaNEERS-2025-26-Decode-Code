package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Blocker extends SubsystemBase{
    ServoEx servoB;
    double naught;
    Telemetry telemetry;
    public Blocker(HardwareMap hwMap, Telemetry telemetry) {
        servoB = new ServoEx(hwMap, "servoB", 0,180);


    }
    public void positionSwitch() {
        if(servoB.ge()!=naught) positionNaught();
        if(servoB.()==naught) positionUp();
    }
    public void positionNaught() {
        servoB.turnToAngle(naught);
    }
    public void positionUp() {
        servoB.turnToAngle(naught-70);
    }
}