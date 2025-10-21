package org.firstinspires.ftc.teamcode.auto;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="RedAuto")
public class RedAuto extends LinearOpMode {
    Drive drivetrain;
    private ElapsedTime runtime = new ElapsedTime();
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredtag = null;
    
    
    @Override
    public void runOpMode(){
        drivetrain = new Drive(hardwareMap,telemetry);
        telemetry.addData("Status","Ready to run auto");
        telemetry.addData("Version","noSlothLoad");
        telemetry.update();
        initAprilTag();



        waitForStart();
        while (opModeIsActive()){
            List<AprilTagDetection> whateverYouWant = aprilTag.getDetections();
            telemetry.addData("AprilTagFound","Not Found");


            for (AprilTagDetection detectedTag : whateverYouWant){

                if (detectedTag.metadata !=null){

                    if (detectedTag.id == 21 ){
                        telemetry.addData("apriltagfound","true");


                    } else {
                        telemetry.addData("apriltagfound","false");

                        
                    }

                }   else {
                    telemetry.addData("AprilTagFound","null tag");
                }

            }
            telemetry.update();
        }
        /*
        runtime.reset();
        while (opModeIsActive()&& (runtime.seconds() < 0.8)) {
            drivetrain.robotCentricDrive(0, -1, 0);
        }
        runtime.reset();
        while (opModeIsActive()&& (runtime.seconds()< 6.7 )){

            drivetrain.robotCentricDrive(0, 0, 1);
        }

         */
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        aprilTag.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
        /*
        class CameraStreamExample extends LinearOpMode{
        OpenCvCamera webcam;
        @Override
        public void runOpMode(){

        int CameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"
        cameraMonitorViewId);
        webcam.setPipeline(new SamplePipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
        @Override
        public void onOpened() {
        webcam.startStreaming(740,480
        

        }
        */

    }
}
