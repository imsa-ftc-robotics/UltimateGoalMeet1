package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

public abstract class Robot extends LinearOpMode {

    public DcMotorEx leftBackDrive;
    public DcMotorEx rightBackDrive;
    public DcMotorEx leftFrontDrive;
    public DcMotorEx rightFrontDrive;

    public DcMotorEx shooter;
    public DcMotorEx wobbleGoalMotor;
    public DcMotorEx intake;
    public DcMotorEx transfer;

    public CRServo intakeWinch;
    public Servo wobbleGoalServo;

    public static final double WOBBLE_CLOSED = 0.75;
    public static final double WOBBLE_OPEN = 0.1;
    public static final double WOBBLE_HALF = 0.5;


    public BNO055IMU imu;

    public Orientation angles;

    public static final int TICKS_PER_INCH = 537 / 12;

    protected boolean initialize_hardware = true;

    public static Robot running_opmode;

    public OpenCvCamera webcam;
    public RingDeterminationPipeline pipeline;

    public void runOpMode(){
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        try {
            running_opmode = this;
            op_mode();
        }
        finally {
            running_opmode = null;
        }
    }
    public void resetDriveEncoders(){
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

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


    public DropPosition getDropPosition() { return pipeline.getDropPosition(); }

    public abstract void op_mode();

    public double getFirstAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public double getSecondAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).secondAngle;
    }

    public double getThirdAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).thirdAngle;
    }

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


    public void stopDrivetrain(){
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

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



}
