package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class DriveTrain extends OpMode {
    private DriveTrain drive;


    DcMotor leftDrive;


    DcMotor rightDrive ;

    DcMotor frontDrive ;

    DcMotor backDrive ;

    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        frontDrive = hardwareMap.get(DcMotor.class, "frontDrive");
        backDrive = hardwareMap.get(DcMotor.class, "backDrive");

    }

    public void loop(){
        double power = -gamepad1.; // separate powers for separate buttons
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        backDrive.setPower(power);
        frontDrive.setPower(power);
    };


  public void moveForward(){


  }
  public void moveLeft(){



  }
  public void moveRight(){




  }
  public void moveBack(){



  }






}
