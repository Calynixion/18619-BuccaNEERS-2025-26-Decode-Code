/*
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BlockerD extends SubsystemBase {
    ServoEx BlockerL,BlockerR;
    Telemetry telemetry;
    //might need to be in radians check later
    private final double launch_angle = 135;
    private final double stop_angle = 90;
    public BlockerD(HardwareMap hardwareMap, Telemetry telemetry) {
        //not sure which one has to be reversed yet
        BlockerL = new ServoEx(hardwareMap,"BlockerL",0,180);
        BlockerR = new ServoEx(hardwareMap,"BlockerR",0,180);
        BlockerL.setInverted(false);
        BlockerR.setInverted(true);

        this.telemetry = telemetry;
        reset();

    }

    public void down(){
        turn(launch_angle);
    }

    public void up(){
        turn(stop_angle);
    }
    public void reset(){
        turn(0);
    }
    public void turn(double angle){
        BlockerL.set(angle);
        BlockerR.set(angle);
    }
}
*/