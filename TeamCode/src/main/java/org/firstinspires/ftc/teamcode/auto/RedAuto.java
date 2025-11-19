package org.firstinspires.ftc.teamcode.auto;

import static com.sun.tools.doclint.Entity.and;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.vision.EOCVAprilTagPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name="RedAuto")
public class RedAuto extends LinearOpMode {
    Drive drivetrain;
    EOCVAprilTagPipeline aprilTagDetectionPipeline;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        drivetrain = new Drive(hardwareMap,telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        aprilTagDetectionPipeline = new EOCVAprilTagPipeline(telemetry);
        aprilTagDetectionPipeline.setDecimation(3);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,600, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });
        telemetry.addData("Status","Ready to run auto");
        telemetry.update();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            ArrayList<AprilTagDetection> detectedTags = aprilTagDetectionPipeline.getLatestDetections();
            if (!detectedTags.isEmpty()) {
                for (AprilTagDetection detectedTag : detectedTags) {
                    telemetry.addData("Pipeline", "Tag " + detectedTag.id + " found");
                }
            }   else {
                telemetry.addData("Pipeline","No tags found");
            }
            telemetry.update();
            if (runtime.seconds()<0.5) {
                drivetrain.robotCentricDrive(0, -1, 0);
            }
        }
    }
}
