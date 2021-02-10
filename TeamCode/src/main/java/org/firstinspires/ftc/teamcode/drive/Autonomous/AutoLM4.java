package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
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

import java.util.ArrayList;

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

        drive.setPoseEstimate(startPose);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        UGContourRingPipeline.Height height = UGContourRingPipeline.Height.ONE;
        while (!isStarted() && !isStopRequested()){
            height = drive.pipeline.getHeight();
            telemetry.addData("[Ring Stack] >>", height);
            telemetry.update();
            idle();
        }


        waitForStart();

        if (isStopRequested()) return;






    }

    private ArrayList<TrajectoryBuilder> nearTrajectory(Pose2d startPose, RobotV3 drive){
        ArrayList<TrajectoryBuilder> near = new ArrayList<TrajectoryBuilder>();

        TrajectoryBuilder near1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(15.0, 45.0), 0.0);

        TrajectoryBuilder near2 = drive.trajectoryBuilder(near1.build().end(), true)
                .splineTo(new Vector2d(0.0, 45.0), 0.0);

        TrajectoryBuilder near3 = drive.trajectoryBuilder(near2.build().end(), true)
                .forward(5)
                .splineTo(new Vector2d(-30.0, 45.0), Math.toRadians(90));

        TrajectoryBuilder near4 = drive.trajectoryBuilder(near3.build().end())
                .strafeLeft(5);

        TrajectoryBuilder near5 = drive.trajectoryBuilder(near4.build().end())
                .splineTo(new Vector2d(8.0, 45.0), 0.0);

        TrajectoryBuilder near6 = drive.trajectoryBuilder(near5.build().end())
                .strafeRight(10);

        near.add(near1);
        near.add(near2);
        near.add(near3);
        near.add(near4);
        near.add(near5);
        near.add(near6);
        return near;
    }

    private void followNear(Pose2d startPose, RobotV3 drive){
        ArrayList<TrajectoryBuilder> traj = nearTrajectory(startPose, drive);

        drive.followTrajectory(traj.get(0).build());
        sleep(500);
        //drive.shooter.setVelocity(2100);
        //put down wobble goal
        drive.followTrajectory(traj.get(1).build());
        sleep(500);
        //bring arm back up
        //run transfer for a few seconds
        drive.followTrajectory(traj.get(2).build());
        sleep(500);
        //bring arm down
        drive.followTrajectory(traj.get(3).build());
        sleep(500);
        //grab wobble goal and bring arm up
        drive.followTrajectory(traj.get(4).build());
        //bring arm down
        drive.followTrajectory(traj.get(5).build());
        sleep(20000);
    }

    private void midTrajectory(Pose2d startPose, RobotV3 drive){
        ArrayList<TrajectoryBuilder> mid = new ArrayList<TrajectoryBuilder>();

        TrajectoryBuilder middle1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-24.0, 18.0), 0.0)
                .splineTo(new Vector2d(40.0, 30.0),0.0);

        TrajectoryBuilder middle2 = drive.trajectoryBuilder(middle1.build().end(), true)
                .splineTo(new Vector2d(0.0, 40.0), 0.0);

        TrajectoryBuilder middle3 = drive.trajectoryBuilder(middle2.build().end());


    }

}