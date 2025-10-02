package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//creates a class for drive systems. Is a subsystem;inherits from subsystem base class provided by ftclib
public class Drive extends SubsystemBase {
    //initialize variables
    MecanumDrive drivetrain;
    MotorEx FrontLeftM,FrontRightM,BackLeftM, BackRightM;
    Telemetry telemetry;
    IMU imu;
    double multiplier = 1;
    double direction = 1;

    //constructor method creates an object of this class
    public Drive(HardwareMap hwMap, Telemetry telemetry){
        //get gyro
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        //gets motor ids from driver station and makes them java objects
        this.FrontLeftM = new MotorEx(hwMap, "FrontLeftM");
        this.FrontRightM = new MotorEx(hwMap, "FrontRightM");
        this.BackLeftM = new MotorEx(hwMap, "BackLeftM");
        this.BackRightM = new MotorEx(hwMap, "BackRightM");

        //sets default motor behavior, will hold position if stopped
        this.FrontLeftM.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.FrontRightM.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.BackLeftM.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.BackRightM.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //build drivetrain object from the 4 motors defined above
        drivetrain = new MecanumDrive(FrontLeftM,FrontRightM,BackLeftM,BackRightM);

        //add telemetry, used for returning values to console output
        this.telemetry = telemetry;
    }

    //methods

    //basic drive method
    public void robotCentricDrive(double x, double y, double rx) {
        drivetrain.driveRobotCentric(x * multiplier * direction, y * multiplier * direction, rx * multiplier * direction);
    }
    public void fieldCentricDrive(double x, double y, double rx) {
        drivetrain.driveFieldCentric(x * multiplier * direction, y * multiplier * direction,rx * multiplier * direction, getFieldYaw());
    }
    public void resetIMU(){
        imu.resetYaw();
    }
    public double getFieldYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
