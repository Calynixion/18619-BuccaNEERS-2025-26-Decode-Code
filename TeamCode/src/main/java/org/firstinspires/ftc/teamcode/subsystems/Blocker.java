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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Blocker extends SubsystemBase{
    SimpleServo servoB;
    SimpleServo servoA;
    public double naught;
    public double angle;
    Telemetry telemetry;
    public Blocker(HardwareMap hwMap, Telemetry telemetry) {
        servoB = new SimpleServo(hwMap, "servoB", 0, 240);
        servoA = new SimpleServo(hwMap, "servoA", 0, 240);
        servoB.setInverted(true);
        servoA.setInverted(false);
    }



    public void positionNaught() {
        /*
        servoB.rotateByAngle(15);
        servoA.rotateByAngle(15);
        */
        servoB.setPosition(0.8);
        servoA.turnToAngle(0.8);
    }
    public void positionUp() {
        /*
        servoB.rotateByAngle(-15);
        servoA.rotateByAngle(-15);
        */
         servoB.setPosition(0);
         servoA.setPosition(0);
    }

    public double get_angle() {
        angle = servoB.getAngle();
        return angle;
    }


}