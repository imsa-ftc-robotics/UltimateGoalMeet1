package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public abstract class RobotV2 extends LinearOpMode {

    //statiing the drive train motors
    public DcMotorEx leftBackDrive;
    public DcMotorEx rightBackDrive;
    public DcMotorEx leftFrontDrive;
    public DcMotorEx rightFrontDrive;
    //declaring the superstructure motors
    public DcMotorEx shooter;
    public DcMotorEx wobbleGoalMotor;
    public DcMotorEx intake;
    public DcMotorEx transfer;
    //declaring servos
    public CRServo intakeWinch;
    public Servo wobbleGoalServo;
    //declaring constants for wobble goal positions
    public static final double WOBBLE_CLOSED = 0.8;
    public static final double WOBBLE_OPEN = 0.1;
    public static final double WOBBLE_HALF = 0.5;

    public Servo transferServo;
    //declaring imu
    public BNO055IMU imu;

    public Orientation angles;
    //declaring the drive motor constants
    public static final int TICKS_PER_INCH = 537 / 12;
    //declaring initializing is true
    protected boolean initialize_hardware = true;
    //holds reference to which opmode is running
    public static RobotV2 running_opmode;

    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam 1"; // insert webcam name from configuration if using webcam

    UGContourRingPipeline pipeline;
    OpenCvCamera camera;

    private int cameraMonitorViewId;


    public void runOpMode(){
        //initialize hardware
        if (initialize_hardware){
            leftBackDrive = (DcMotorEx)hardwareMap.get("backLeft");
            leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
            leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            rightBackDrive = (DcMotorEx)hardwareMap.get("backRight");
            rightBackDrive.setDirection((DcMotorEx.Direction.FORWARD));
            rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            leftFrontDrive = (DcMotorEx)hardwareMap.get("frontLeft");
            leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
            leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            rightFrontDrive = (DcMotorEx)hardwareMap.get("frontRight");
            rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
            rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            wobbleGoalMotor =(DcMotorEx)hardwareMap.get("wobbleGoal");
            wobbleGoalMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            intake = (DcMotorEx)hardwareMap.get("intake");
            intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            shooter = (DcMotorEx)hardwareMap.get("shooter");
            shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            transfer = (DcMotorEx)hardwareMap.get("transfer");
            transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            transferServo = (Servo)hardwareMap.get("transferServo");
            wobbleGoalServo =(Servo)hardwareMap.get("wobbleGoalServo");
            wobbleGoalServo.setDirection(Servo.Direction.REVERSE);
            intakeWinch = (CRServo)hardwareMap.get("intakeWinch");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(75, 0, 10, 15);
            shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

 /*           int target_tollerance = 10;
            double p = 7.2;
            leftBackDrive.setVelocityPIDFCoefficients(12.20372439*0.1, 12.20372439*0.01, 0, 12.20372439);
            leftBackDrive.setPositionPIDFCoefficients(p);
            leftBackDrive.setTargetPositionTolerance(target_tollerance);

            leftFrontDrive.setVelocityPIDFCoefficients(12.20372439*0.1, 12.20372439*0.01, 0, 12.20372439);
            leftFrontDrive.setPositionPIDFCoefficients(p);
            leftFrontDrive.setTargetPositionTolerance(target_tollerance);

            rightBackDrive.setVelocityPIDFCoefficients(12.20372439*0.1, 12.20372439*0.01, 0, 12.20372439);
            rightBackDrive.setPositionPIDFCoefficients(p);
            rightBackDrive.setTargetPositionTolerance(target_tollerance);

            rightFrontDrive.setVelocityPIDFCoefficients(12.20372439*0.1, 12.20372439*0.01, 0, 12.20372439);
            rightFrontDrive.setPositionPIDFCoefficients(p);
            rightFrontDrive.setTargetPositionTolerance(target_tollerance);*/
        }
        //intializing the webcam from the hardwareMap
        cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        try {
            running_opmode = this;
            op_mode();
        }
        finally {
            running_opmode = null;
        }
    }
    //resets the motor encoder on the drive train
    public void resetDriveEncoders(){
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    //moves the drive train motors to a set position
    public void moveToPosition(double motorPower, int ticks){
        resetDriveEncoders();

        double motorVelocity = motorPower*2700;

        leftBackDrive.setTargetPosition(ticks);
        leftFrontDrive.setTargetPosition(ticks);
        rightBackDrive.setTargetPosition(ticks);
        rightFrontDrive.setTargetPosition(ticks);

        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        rightFrontDrive.setVelocity(motorVelocity);
        rightBackDrive.setVelocity(motorVelocity);
        leftBackDrive.setVelocity(motorVelocity);
        leftFrontDrive.setVelocity(motorVelocity);

        while(leftBackDrive.isBusy() && !isStopRequested()){
            telemetry.addData("Status", "not there yet");
            telemetry.addData("left back", leftBackDrive.getCurrentPosition());
            telemetry.addData("left front", leftFrontDrive.getCurrentPosition());
            telemetry.addData("right back", rightBackDrive.getCurrentPosition());
            telemetry.addData("right front", rightFrontDrive.getCurrentPosition());
            telemetry.addData("busy1", leftBackDrive.isBusy());
            telemetry.addData("busy2", leftFrontDrive.isBusy());
            telemetry.addData("busy3", rightFrontDrive.isBusy());
            telemetry.addData("busy4", rightBackDrive.isBusy());
            telemetry.addData("Target", leftBackDrive.getTargetPosition());
            telemetry.update();
        }

        stopDrivetrain();

    }

    //gets where to drop wobble goal
    public abstract void op_mode();
    //heading of IMU
    public double getFirstAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public double getSecondAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).secondAngle;
    }

    public double getThirdAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).thirdAngle;
    }
    //PID algorithim to set robot at specific heading
    public void reorientIMU(double targetAngle, double left, double right, double threshold, double kp, double ki, double kd) {
        //get the current value in radians
        double currentValue = getFirstAngle();
        //convert the target to radians
        targetAngle = Math.toRadians(targetAngle);
        //initialize PID variables
        double error;
        double derivative;
        double integral = 0;
        double lastError = 0;
        double output;
        //convert the threshold to radians
        threshold = Math.toRadians(threshold);
        useEncoders();
        while (Math.abs(targetAngle - currentValue) > threshold && opModeIsActive()) {
            //the error (aka proportional) is the difference between set point and current point
            error = targetAngle- currentValue;
            //integral is the summation of all the past error
            integral += error;
            //derivative is the difference between current and past error
            //tries to predict future error
            derivative = error - lastError;
            //multiply each value by their respective constants and sum to get outuput
            output = (error * kp) + (integral * ki) + (derivative * kd);

            //set motor power based output value
            leftFrontDrive.setPower(output * left);
            leftBackDrive.setPower(output * left);
            rightFrontDrive.setPower(output * right);
            rightBackDrive.setPower(output * right);

            //get the current value from the IMU
            currentValue = getFirstAngle();
            telemetry.addData("Current Value", currentValue);
            telemetry.addData("Target", targetAngle);
            telemetry.addData("Left Power", leftBackDrive.getPower());
            telemetry.addData("Right Power", rightBackDrive.getPower());
            telemetry.update();
            //make the last error equal to the current error
            lastError = error;
        }
          stopDrivetrain();
    }
    //strafe right or left for a specified amount of time
    public void strafe(double power, int sleepTime){
        resetDriveEncoders();

        useEncoders();

        leftBackDrive.setPower(-power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);

        sleep(sleepTime);

        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    //kills the drive train motors
    public void stopDrivetrain(){
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    //run mode to use encoder
    public void useEncoders(){
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    //move using encoder for a specified amount of time
    public void moveWithEncoders(double motorPower, int sleepTime){
        useEncoders();

        leftFrontDrive.setPower(motorPower);
        leftBackDrive.setPower(motorPower);
        rightBackDrive.setPower(motorPower);
        rightFrontDrive.setPower(motorPower);

        sleep(sleepTime);

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }
        //strafe at the specified angle at specified time
    public void strafeAngle(double speed, double degrees, int sleepTime){
        //set runmode to RUN_USING_ENCODERS
        useEncoders();

        //convert angle to radians
        double radians = Math.toRadians(degrees);

        //subtract pi/4 because the rollers are angled pi/4 radians
        double robotAngle = radians - (Math.PI/4);
        double[] motorPower;
        motorPower = new double[4];

        //set motor powers based on the specified angle
        motorPower[0] = Math.cos(robotAngle);
        motorPower[1] = Math.sin(robotAngle);
        motorPower[2] = Math.sin(robotAngle);
        motorPower[3] = Math.cos(robotAngle);


        //because of limitations with the sin and cos functions, the motors are not always going at the speed that is specified
        //in order to do this, we multiply each motor power by the desired speed over the highest motor power
        double maxPower = 0;
        for (double power : motorPower) {
            if (Math.abs(power) > maxPower) {
                maxPower = Math.abs(power);
            }
        }

        double ratio;

        if (maxPower == 0) {
            ratio = 0;
        } else {
            ratio = speed / maxPower;
        }

        double leftFront = Range.clip((ratio * motorPower[0]), -1, 1);
        double rightFront = Range.clip((ratio * motorPower[1]), -1, 1);
        double leftBack = Range.clip((ratio * motorPower[2]) , -1, 1);
        double rightBack = Range.clip((ratio * motorPower[3]), -1, 1);

        //set motor powers
        leftFrontDrive.setPower(leftFront);
        rightFrontDrive.setPower(rightFront);
        leftBackDrive.setPower(leftBack);
        rightBackDrive.setPower(rightBack);

        sleep(sleepTime);

        stopDrivetrain();
    }
    //using PID algorithm to make sure it strafes in a straight line
    public void strafingPID(double motorPower, double sleepTime, double kp, double ki, double kd){
        double targetAngle = getFirstAngle();
        double targetTime = getRuntime()+(sleepTime/1000);


        leftBackDrive.setPower(-motorPower);
        leftFrontDrive.setPower(motorPower);
        rightBackDrive.setPower(motorPower);
        rightFrontDrive.setPower(-motorPower);

        double error = 0;
        double integral = 0;
        double derivative = 0;
        double lastError = 0;
        double outputChange;

        //measuring the error and integral and sets the motor power based on the PID
        while ((getRuntime()<targetTime)&&!isStopRequested()){
            error = targetAngle-getFirstAngle();
            integral += error;
            derivative = error-lastError;
            outputChange = (error*kp)+(integral*ki)+(derivative*kd);


            leftBackDrive.setPower(-motorPower-outputChange);
            leftFrontDrive.setPower(motorPower-outputChange);
            rightBackDrive.setPower(motorPower+outputChange);
            rightFrontDrive.setPower(-motorPower+outputChange);

            lastError = error;

        }

        stopDrivetrain();
    }
    public void wrapIMU(double targetAngle, double left, double right, double threshold, double kp, double ki, double kd) {
        //get the current value in radians
        double currentValue = getFirstAngleWrapped();
        //convert the target to radians
        targetAngle = Math.toRadians(targetAngle);
        //initialize PID variables
        double error;
        double derivative;
        double integral = 0;
        double lastError = 0;
        double output;
        //convert the threshold to radians
        threshold = Math.toRadians(threshold);
        useEncoders();
        while (Math.abs(targetAngle - currentValue) > threshold && opModeIsActive()) {
            //the error (aka proportional) is the difference between set point and current point
            error = targetAngle- currentValue;
            //integral is the summation of all the past error
            integral += error;
            //derivative is the difference between current and past error
            //tries to predict future error
            derivative = error - lastError;
            //multiply each value by their respective constants and sum to get outuput
            output = (error * kp) + (integral * ki) + (derivative * kd);

            //set motor power based output value
            leftFrontDrive.setPower(output * left);
            leftBackDrive.setPower(output * left);
            rightFrontDrive.setPower(output * right);
            rightBackDrive.setPower(output * right);

            //get the current value from the IMU
            currentValue = getFirstAngleWrapped()
            ;
            telemetry.addData("Current Value", currentValue);
            telemetry.addData("Target", targetAngle);
            telemetry.addData("Left Power", leftBackDrive.getPower());
            telemetry.addData("Right Power", rightBackDrive.getPower());
            telemetry.update();
            //make the last error equal to the current error
            lastError = error;
        }
        stopDrivetrain();
    }
    public double getFirstAngleWrapped(){
        if (getFirstAngle() < 0) {
            return getFirstAngle() + (Math.PI*2);
        }
        else{
            return getFirstAngle();
        }
    }




}
