package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.shoot_3;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Bot_Trigger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.vision.EOCVAprilTagPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
@Autonomous(name = "AutoBlueNew", group = "Autos")
public class DecodeBlueNew extends CommandOpMode {
    EOCVAprilTagPipeline aprilTagDetectionPipeline;
    private static Follower follower;
    private TelemetryManager telemetryM;
    private Shooter shooter;
    private Intake intake;
    private Bot_Trigger trigger;
    public double shoot_power = 1;
    public double intake_power = 1;
    public static int tag;
    public boolean looking=true;
    public PathChain Path1, Path2, Path3, Path4, Path67;
    public static final Pose shootPoseBlue = new Pose(48,96,Math.toRadians(315));
    public static final Pose cameraPoseBlue = new Pose(48,96,Math.toRadians(230));
    public static final Pose startPoseBlue = new Pose(20.6,122.7,Math.toRadians(321.46));
    private InstantCommand shoot_3() {
        return new InstantCommand(() -> {
            shooter.spin(shoot_power);
        });
    }
    private InstantCommand intake_start() {
        return new InstantCommand(() -> {
            intake.spin();
        });
    }
    private InstantCommand intake_stop() {
        return new InstantCommand(() -> {
            intake.stop();
        });
    }
    public void buildPaths(){
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPoseBlue,cameraPoseBlue)
                )
                .setConstantHeadingInterpolation(cameraPoseBlue.getHeading())
                .build();
        Path67 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPoseBlue, shootPoseBlue)
                )
                .setLinearHeadingInterpolation(startPoseBlue.getHeading(), shootPoseBlue.getHeading())
                .build();
        if (tag==21) {
            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 96.000), new Pose(48.000, 35.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 35.500), new Pose(14.000, 35.500))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(14.000, 35.500),
                                    new Pose(51.500, 30.000),
                                    new Pose(48.000, 96.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .build();
        } else if (tag==22) {
            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 96.000), new Pose(48.000, 59.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 59.500), new Pose(19.000, 59.500))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(19.000, 59.500),
                                    new Pose(51.500, 59.500),
                                    new Pose(48.000, 96.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .build();
        } else if (tag==23) {
            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 96.000), new Pose(48.000, 83.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.000, 83.500), new Pose(19.000, 83.500))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(19.000, 83.500),
                                    new Pose(51.500, 83.500),
                                    shootPoseBlue
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), shootPoseBlue.getHeading())
                    .build();

        }
    }

    @Override
    public void initialize() {
        super.reset();
        CommandScheduler.getInstance().reset();

        tag=0;
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
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPoseBlue);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shooter(hardwareMap,telemetry);
        intake = new Intake(hardwareMap, telemetry);
        trigger = new Bot_Trigger(hardwareMap, telemetry);
        buildPaths();

        SequentialCommandGroup autoSequence = new SequentialCommandGroup(
            new FollowPathCommand(follower,Path67),
            shoot_3(),
            new WaitCommand(4000),
            new FollowPathCommand(follower,Path1,true),
            new WaitCommand(3000),
            new FollowPathCommand(follower,Path2),
            intake_start(),
            new FollowPathCommand(follower,Path3),
            intake_stop(),
            new FollowPathCommand(follower,Path4,true),
            shoot_3()
        );

        register(shooter,intake,trigger);

        schedule(autoSequence);
    }


    @Override
    public void run() {
        super.run();
        CommandScheduler.getInstance().run();
        ArrayList<AprilTagDetection> detectedTags = aprilTagDetectionPipeline.getLatestDetections();
        if (!detectedTags.isEmpty()) {
            for (AprilTagDetection detectedTag : detectedTags) {
                if((detectedTag.id<=23) && (detectedTag.id>=21) && looking){
                    tag=detectedTag.id;
                    looking=false;
                    buildPaths();
                }
            }
        }
        telemetry.addData("tag",tag);
        telemetry.update();

    }
}
