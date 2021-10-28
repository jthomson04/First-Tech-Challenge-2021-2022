package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "Autonomous", group = "Autonomous")
public class Auto extends LinearOpMode {

    private static final String TENSORFLOW_ASSET_NAME = "FreightFrenzy_DM.tflite";
    private static final String[] LABELS = {
            "Duck",
            "Marker",
    };
    private static final String VUFORIA_KEY = "AcRDh/L/////AAABmSXgpqzx70p0vDh2eqv9N6YEI6fwUF4tbXNAvrYXcMt+Zbq6qXM2z4Aq7KnWGblN2ZiCjGIpLlWeXL7IQrtvBJFzPNil285nzLsLg/KWynX1Pss7RKL7i/O4hFxsorVD/+4kMvkFMV7q1uVt4mY4d+SuChH3vAQA6t5NnVJGhh6M+eAeLcQYTF9KCkNL0xgXYeg06BPbppTydDgNRqTrsGwZgegIHutSHH89R/P1NdR9arRifjrfUtNEoIHglPMJ7Mh3PeFH3CpcTBfdgCuYcQCZb0lGAMI8v0Nlwh6lHkRmUFjQsfR+ujiiAAx0agouc2mEy1dK/lLDq34ZtcoAqNEpI1zinV8lkpVvFE3y9xL4";
    private MecanumDrive drive;
    private float cumulativeAngle = 0;
    private float prevAngle = 0;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private List<Recognition> objectRecognitions;

    @Override
    public void runOpMode() {

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        DcMotor backLeft = hardwareMap.dcMotor.get("backleft");
        DcMotor backRight = hardwareMap.dcMotor.get("backright");
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontleft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontright");


        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight, imu);

        initCV();


        waitForStart();

        /*gyroTurn(360);
        delay(2000);
        gyroTurn(0);
        gyroForward(0.2, 1000, 0);
        gyroTurn(180);
        gyroForward(0.2, 1000, 180);
        gyroTurn(0);*/

        while (opModeIsActive()) {
            updateRecognitions();
            displayRecognitions();
            telemetry.update();
        }


    }

    private void showOrientation() {
        telemetry.addData("Orientation", cumulativeAngle);
    }

    private void gyroTurn(double desiredAngle, double turnPower, double closeRangePower, double slowDownCutoff) {
        // gyroscopic turning for autonomous
        updateOrientation();
        float initialAngle = cumulativeAngle;

        boolean right = desiredAngle > cumulativeAngle;

        float turningMultiplier = right ? -1 : 1;

        while (opModeIsActive()) {
            updateOrientation();
            double percentageThrough = mapValue(cumulativeAngle, initialAngle, desiredAngle, 0, 1);
            double power = percentageThrough > slowDownCutoff ? closeRangePower : turnPower;

            showOrientation();
            telemetry.addData("Power", power);
            telemetry.addData("Percentage Through Turn", percentageThrough);
            telemetry.update();

            drive.setMotorPowers(power * -1 * turningMultiplier, power * turningMultiplier, power * -1 * turningMultiplier, power * turningMultiplier);
            if (Math.abs(desiredAngle - cumulativeAngle) < 2) {
                break;
            }
        }
        drive.setMotorPowers(0, 0, 0, 0);
    }

    private void gyroTurn(double desiredAngle) {
        gyroTurn(desiredAngle, 0.8, 0.2, 0.8);
    }


    private void gyroForward(double power, int millis, float maintainAngle) {
        long endTime = SystemClock.elapsedRealtime() + millis;
        while (opModeIsActive()) {
            updateOrientation();

            double error = maintainAngle - cumulativeAngle;
            double offset = error * 0.01;

            showOrientation();
            telemetry.addData("Power Offset", offset);
            telemetry.addData("Time Remaining", endTime - SystemClock.elapsedRealtime());
            telemetry.update();

            drive.setMotorPowers(power + offset, power - offset, power, power);
            if (SystemClock.elapsedRealtime() >= endTime) {
                break;
            }
        }
    }


    private void delay(int millis) {
        long initTime = SystemClock.elapsedRealtime();
        long endTime = initTime + millis;
        while (opModeIsActive()) {
            if (SystemClock.elapsedRealtime() > endTime) {
                break;
            }
        }
    }

    private void updateOrientation() {
        float currentAngle = processAngle(drive.getOrientation().thirdAngle);
        telemetry.addData("processedangle", currentAngle);
        if (Math.abs(prevAngle - currentAngle) > 300) {  //checks if angle has wrapped around
            if (currentAngle < 180) { // crossed from 360 to 0
                this.cumulativeAngle += (360 - prevAngle) + currentAngle;
            } else { // crossed from 0 to 360
                this.cumulativeAngle -= (360 - currentAngle) + prevAngle;
            }
        } else {
            this.cumulativeAngle += currentAngle - prevAngle;
        }
        this.prevAngle = currentAngle;
    }


    private float processAngle(float angle) {
        angle = angle * -1;
        return (angle < 0) ? angle + 360 : angle;
    }

    // from https://stackoverflow.com/questions/7505991/arduino-map-equivalent-function-in-java
    double mapValue(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    private void initCV() {
        initCV(1.5, 16.0 / 9.0);
    }

    private void initCV(double magnification, double aspectRatio) {
        telemetry.addData("Computer Vision", "Initializing");
        telemetry.update();
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.65f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TENSORFLOW_ASSET_NAME, LABELS);

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(magnification, aspectRatio);
        }
        telemetry.addData("Computer Vision", "Activated");
        telemetry.update();

    }

    private void updateRecognitions() {
        if (tfod != null) {
            List<Recognition> currentRecognitions = tfod.getUpdatedRecognitions();
            if (currentRecognitions != null) {
                objectRecognitions = currentRecognitions;
            }
        }
    }

    private void displayRecognitions() {
        int i = 0;
        if (objectRecognitions != null) {
            for (Recognition recognition : objectRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                i++;
            }
        }
    }
}




