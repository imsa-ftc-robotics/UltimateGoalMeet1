package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.RobotV3;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
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

        Pose2d startPose = new Pose2d(-64.0, 24.0, 0.0);


        waitForStart();

        if (isStopRequested()) return;

        TrajectoryBuilder near1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(15.0, 50.0), 0.0);

        drive.followTrajectory(near1.build());
    }

}
