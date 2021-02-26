package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Archive.Robot;
import org.firstinspires.ftc.teamcode.RegionalsBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.ServiceLoader;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

@Autonomous
public class AutoRegionals extends LinearOpMode {

    final int AUTONOMOUS_SHOOTER_VELOCITY = 2100;

    @Override
    public void runOpMode() throws InterruptedException {
        RegionalsBot drive = new RegionalsBot(hardwareMap);
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

        drive.wobbleGoalServo.setPosition(drive.WOBBLE_CLOSED);
        drive.transferServo.setPosition(0.31);


        waitForStart();

        if (isStopRequested()) return;

        switch (height){
            case ZERO:
                followNear(startPose, drive);
                break;

            case ONE:
                followMid(startPose, drive);
                break;

            case FOUR:
                followFar(startPose, drive);
                break;
        }

    }

    private ArrayList<TrajectoryBuilder> nearTrajectory(Pose2d startPose, RegionalsBot drive){
        ArrayList<TrajectoryBuilder> near = new ArrayList<TrajectoryBuilder>();

        TrajectoryBuilder near1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(15.0, 45.0), 0.0)
                .addTemporalMarker(0.0, () -> {
                    drive.intakeWinch.setPower(1);
                })
                .addTemporalMarker(2.5, () -> {
                    drive.intakeWinch.setPower(0);
                });

        TrajectoryBuilder nearStrafe = drive.trajectoryBuilder(near1.build().end())
                .strafeRight(5);

        TrajectoryBuilder near2 = drive.trajectoryBuilder(nearStrafe.build().end(), true)
                .splineTo(new Vector2d(-2.0, 38.0), Math.toRadians(180.0))
                .addTemporalMarker(0.2, () -> {
                    drive.wobbleGoalMotor2.setPower(0.9);
                    drive.wobbleGoalMotor1.setPower(0.9);
                })
                .addTemporalMarker(1, () -> {
                    drive.wobbleGoalMotor2.setPower(0);
                    drive.wobbleGoalMotor1.setPower(0);
                });

        //turn 180

        TrajectoryBuilder near3 = drive.trajectoryBuilder(near2.build().end().plus(new Pose2d(0,0, Math.toRadians(180))), true)
                .forward(5)
                .splineTo(new Vector2d(-30.0, 45.0), Math.toRadians(90));

        TrajectoryBuilder near4 = new TrajectoryBuilder(near3.build().end(), new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(35, TRACK_WIDTH))), drive.accelConstraint)
                .strafeLeft(8);


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
        near.add(nearStrafe);
        return near;
    }

    private ArrayList<TrajectoryBuilder> midTrajectory(Pose2d startPose, RegionalsBot drive){
        ArrayList<TrajectoryBuilder> mid = new ArrayList<TrajectoryBuilder>();

        TrajectoryBuilder middle1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-24.0, 18.0), 0.0)
                .splineTo(new Vector2d(38.0, 18.0),0.0)
                .addTemporalMarker(0.0, () -> {
                    drive.intakeWinch.setPower(1);
                })
                .addTemporalMarker(2.5, () -> {
                    drive.intakeWinch.setPower(0);
                });

        TrajectoryBuilder middleStrafe = drive.trajectoryBuilder(middle1.build().end())
                .strafeRight(8);


        TrajectoryBuilder middle2 = drive.trajectoryBuilder(middleStrafe.build().end(), true)
                .splineTo(new Vector2d(0.0, 40.0), 0.0)
                .addTemporalMarker(0.2, () -> {

                    drive.wobbleGoalMotor2.setPower(0.9);
                    drive.wobbleGoalMotor1.setPower(0.9);
                })
                .addTemporalMarker(1, () -> {
                    drive.wobbleGoalMotor2.setPower(0);
                    drive.wobbleGoalMotor1.setPower(0);
                });

        TrajectoryBuilder middle3 = drive.trajectoryBuilder(middle2.build().end())
                .splineTo(new Vector2d(-10.0, 47.0), Math.toRadians(180))
                .splineTo(new Vector2d(-30.0, 47.0), Math.toRadians(180));

        //TURN AFTERWARDS

        TrajectoryBuilder middle4 = new TrajectoryBuilder(middle3.build().end().plus(new Pose2d(0,0, Math.toRadians(270))), new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(35, TRACK_WIDTH))), drive.accelConstraint)
                .strafeLeft(10);



        TrajectoryBuilder middle5 = drive.trajectoryBuilder(middle4.build().end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .splineTo(new Vector2d(27.0, 18.0), 0.0);

        TrajectoryBuilder middle6 = drive.trajectoryBuilder(middle5.build().end())
                .strafeRight(10)
                .splineToConstantHeading(new Vector2d(7, 3), 0.0);

        mid.add(middle1);
        mid.add(middle2);
        mid.add(middle3);
        mid.add(middle4);
        mid.add(middle5);
        mid.add(middle6);
        mid.add(middleStrafe);

        return mid;
    }

    private ArrayList<TrajectoryBuilder> farTrajectory(Pose2d startPose, RegionalsBot drive){
        ArrayList<TrajectoryBuilder> far = new ArrayList<TrajectoryBuilder>();

        TrajectoryBuilder far1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-32.0, 18.0), 0.0)
                .splineTo(new Vector2d(58.0, 48.0), 0.0)
                .addTemporalMarker(0.0, () -> {
                    drive.intakeWinch.setPower(1);
                })
                .addTemporalMarker(2.5, () -> {
                    drive.intakeWinch.setPower(0);
                });
//drop wobble

        TrajectoryBuilder farStrafe = drive.trajectoryBuilder(far1.build().end())
                .strafeRight(10);

        TrajectoryBuilder far2 = drive.trajectoryBuilder(farStrafe.build().end(), true)
                .splineTo(new Vector2d(0.0, 40.0), 0.0);
//shoot

        TrajectoryBuilder far3 = drive.trajectoryBuilder(far2.build().end(), true)
                .splineTo(new Vector2d(-30.0, 48.0), Math.toRadians(180));

        //TURN 90 HERE

        TrajectoryBuilder far4 = drive.trajectoryBuilder(far3.build().end().plus(new Pose2d(0,0,Math.toRadians(90))))
                .strafeLeft(10.0);

        TrajectoryBuilder far5 = drive.trajectoryBuilder(far4.build().end())
                .splineTo(new Vector2d(-20.0, 50.0), 0.0)
                .splineTo(new Vector2d(58.0, 42.0), 0.0)
                .addTemporalMarker(0.5, () -> {
                    drive.wobbleGoalMotor2.setPower(0);
                    drive.wobbleGoalMotor1.setPower(0);
                });

        TrajectoryBuilder far6 = drive.trajectoryBuilder(far5.build().end())
                .strafeRight(5)
                .splineToConstantHeading(new Vector2d(10, 37), 0.0);

        far.add(far1);
        far.add(far2);
        far.add(far3);
        far.add(far4);
        far.add(far5);
        far.add(far6);
        far.add(farStrafe);

        return far;

    }

    private void followNear(Pose2d startPose, RegionalsBot drive){
        ArrayList<TrajectoryBuilder> traj = nearTrajectory(startPose, drive);

        drive.followTrajectory(traj.get(0).build());
        sleep(200);

        //put down wobble goal

        drive.wobbleGoalMotor1.setPower(-1);
        drive.wobbleGoalMotor2.setPower(-1);
        sleep(1000);
        drive.wobbleGoalMotor2.setPower(0);
        drive.wobbleGoalMotor1.setPower(0);
        drive.wobbleGoalServo.setPosition(drive.WOBBLE_OPEN);
        telemetry.addData("done", "done");
        telemetry.update();

        drive.followTrajectory(traj.get(6).build());
        sleep(100);

        drive.followTrajectoryAsync(traj.get(1).build());
        drive.drive_with_shooter_pid(AUTONOMOUS_SHOOTER_VELOCITY);
        drive.wait_with_shooter_pid(AUTONOMOUS_SHOOTER_VELOCITY, 500);

        drive.turnAsync(Math.toRadians(180));
        drive.drive_with_shooter_pid(AUTONOMOUS_SHOOTER_VELOCITY);
        drive.wait_with_shooter_pid(AUTONOMOUS_SHOOTER_VELOCITY, 500);

        drive.transferServo.setPosition(0);
        drive.wait_with_shooter_pid(AUTONOMOUS_SHOOTER_VELOCITY, 300);
        drive.transfer.setPower(-0.9);
        drive.intake.setPower(-0.5);
        drive.wait_with_shooter_pid(AUTONOMOUS_SHOOTER_VELOCITY, 3000);
        drive.transfer.setPower(0);
        drive.shooter1.setPower(0);
        drive.shooter2.setPower(0);
        drive.intake.setPower(0);


        drive.followTrajectory(traj.get(2).build());


        drive.wobbleGoalMotor2.setPower(-0.9);
        drive.wobbleGoalMotor1.setPower(-0.9);
        sleep(900);
        drive.wobbleGoalMotor2.setPower(0);
        drive.wobbleGoalMotor1.setPower(0);
        drive.wobbleGoalServo.setPosition(drive.WOBBLE_OPEN);

        //bring arm down
        drive.followTrajectory(traj.get(3).build());
        sleep(500);
        //grab wobble goal and bring arm up
        drive.wobbleGoalServo.setPosition(drive.WOBBLE_CLOSED);
        sleep(500);
        drive.wobbleGoalMotor2.setPower(1);
        drive.wobbleGoalMotor1.setPower(1);
        sleep(1100);
        drive.wobbleGoalMotor2.setPower(0);
        drive.wobbleGoalMotor1.setPower(0);
        sleep(500);

        drive.followTrajectory(traj.get(4).build());
        //bring arm down
        drive.wobbleGoalMotor2.setPower(-0.9);
        drive.wobbleGoalMotor1.setPower(-0.9);
        sleep(800);
        drive.wobbleGoalMotor2.setPower(0);
        drive.wobbleGoalMotor1.setPower(0);
        drive.wobbleGoalServo.setPosition(drive.WOBBLE_OPEN);
        sleep(500);

        drive.followTrajectory(traj.get(5).build());
        sleep(20000);
    }

    private void followMid(Pose2d startPose, RegionalsBot drive){
        ArrayList<TrajectoryBuilder> traj = midTrajectory(startPose, drive);

        drive.followTrajectory(traj.get(0).build());

        //put down wobble goal
        drive.wobbleGoalMotor2.setPower(-0.9);
        drive.wobbleGoalMotor1.setPower(-0.9);
        sleep(800);
        drive.wobbleGoalMotor2.setPower(0);
        drive.wobbleGoalMotor1.setPower(0);
        sleep(100);
        drive.wobbleGoalServo.setPosition(drive.WOBBLE_OPEN);
        sleep(300);

        drive.followTrajectory(traj.get(6).build());

        drive.followTrajectoryAsync(traj.get(1).build());
        drive.drive_with_shooter_pid(AUTONOMOUS_SHOOTER_VELOCITY);
        drive.transferServo.setPosition(0);
        drive.wait_with_shooter_pid(AUTONOMOUS_SHOOTER_VELOCITY, 300);

        //run transfer and shoot
        drive.transfer.setPower(-0.9);
        drive.intake.setPower(-0.5);
        drive.wait_with_shooter_pid(AUTONOMOUS_SHOOTER_VELOCITY, 3000);
        drive.transfer.setPower(0);
        drive.shooter1.setPower(0);
        drive.shooter2.setPower(0);
        drive.intake.setPower(0);

        drive.followTrajectory(traj.get(2).build());
        sleep(500);
        drive.turn(Math.toRadians(-90));
        //bring wobble down

        drive.wobbleGoalMotor2.setPower(-0.9);
        drive.wobbleGoalMotor1.setPower(-0.9);
        sleep(800);
        drive.wobbleGoalMotor2.setPower(0);
        drive.wobbleGoalMotor1.setPower(0);
        drive.wobbleGoalServo.setPosition(drive.WOBBLE_OPEN);

        drive.followTrajectory(traj.get(3).build());
        //grab
        drive.wobbleGoalServo.setPosition(drive.WOBBLE_CLOSED);
        sleep(700);
        drive.wobbleGoalMotor2.setPower(1);
        drive.wobbleGoalMotor1.setPower(1);
        sleep(1000);
        drive.wobbleGoalMotor2.setPower(0);
        drive.wobbleGoalMotor1.setPower(0);

        drive.turn(Math.toRadians(-90));
        sleep(500);
        drive.followTrajectory(traj.get(4).build());
        //drop wobble
        drive.wobbleGoalMotor2.setPower(-0.9);
        drive.wobbleGoalMotor1.setPower(-0.9);
        sleep(800);
        drive.wobbleGoalMotor2.setPower(0);
        drive.wobbleGoalMotor1.setPower(0);
        drive.wobbleGoalServo.setPosition(drive.WOBBLE_OPEN);
        sleep(500);
        drive.followTrajectory(traj.get(5).build());
    }

    private void followFar(Pose2d startPose, RegionalsBot drive) {
        ArrayList<TrajectoryBuilder> traj = farTrajectory(startPose, drive);

        drive.followTrajectory(traj.get(0).build());

        drive.wobbleGoalMotor2.setPower(-0.9);
        drive.wobbleGoalMotor1.setPower(-0.9);
        sleep(800);
        drive.wobbleGoalMotor2.setPower(0);
        drive.wobbleGoalMotor1.setPower(0);
        drive.wobbleGoalServo.setPosition(drive.WOBBLE_OPEN);
        sleep(200);

        drive.followTrajectory(traj.get(6).build());

        drive.followTrajectory(traj.get(1).build());
        drive.drive_with_shooter_pid(AUTONOMOUS_SHOOTER_VELOCITY);

        drive.transferServo.setPosition(0);
        sleep(300);
        drive.transfer.setPower(-0.9);
        drive.intake.setPower(-0.5);
        sleep(3000);
        drive.transfer.setPower(0);
        drive.shooter1.setPower(0);
        drive.shooter2.setPower(0);
        drive.intake.setPower(0);

        drive.followTrajectory(traj.get(2).build());
        drive.turn(Math.toRadians(90));


        drive.followTrajectory(traj.get(3).build());
        //grab wobble
        drive.wobbleGoalServo.setPosition(drive.WOBBLE_CLOSED);
        sleep(700);
        drive.wobbleGoalMotor2.setPower(1);
        drive.wobbleGoalMotor1.setPower(1);
        sleep(400);

        drive.followTrajectory(traj.get(4).build());
        //drop wobble
        drive.wobbleGoalMotor2.setPower(-0.9);
        drive.wobbleGoalMotor1.setPower(-0.9);
        sleep(700);
        drive.wobbleGoalMotor2.setPower(0);
        drive.wobbleGoalMotor1.setPower(0);
        drive.wobbleGoalServo.setPosition(drive.WOBBLE_OPEN);
        drive.followTrajectory(traj.get(5).build());

    }
}