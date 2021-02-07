package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.RobotV3;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class AutoLM4 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotV3 drive = new RobotV3(hardwareMap);
        drive.cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (drive.USING_WEBCAM) {
            drive.camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, drive.WEBCAM_NAME), drive.cameraMonitorViewId);
        } else {
            drive.camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, drive.cameraMonitorViewId);
        }

        drive.camera.setPipeline(drive.pipeline = new UGContourRingPipeline(telemetry, drive.DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(drive.CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(drive.HORIZON);

        drive.camera.openCameraDeviceAsync(() -> drive.camera.startStreaming(drive.CAMERA_WIDTH, drive.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        waitForStart();

        if (isStopRequested()) return;

        Trajectory near1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        drive.followTrajectory(near1);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(near1.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
    }

}
