package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Disabled
public class Auto extends LinearOpMode {

    private static final String TENSORFLOW_ASSET_NAME = "FreightFrenzy_BCDM.tflite"; // name of presaved bundle tflite model
    private static final String[] LABELS = { // labels for tflite model
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY = "AcRDh/L/////AAABmSXgpqzx70p0vDh2eqv9N6YEI6fwUF4tbXNAvrYXcMt+Zbq6qXM2z4Aq7KnWGblN2ZiCjGIpLlWeXL7IQrtvBJFzPNil285nzLsLg/KWynX1Pss7RKL7i/O4hFxsorVD/+4kMvkFMV7q1uVt4mY4d+SuChH3vAQA6t5NnVJGhh6M+eAeLcQYTF9KCkNL0xgXYeg06BPbppTydDgNRqTrsGwZgegIHutSHH89R/P1NdR9arRifjrfUtNEoIHglPMJ7Mh3PeFH3CpcTBfdgCuYcQCZb0lGAMI8v0Nlwh6lHkRmUFjQsfR+ujiiAAx0agouc2mEy1dK/lLDq34ZtcoAqNEpI1zinV8lkpVvFE3y9xL4";
    BNO055IMU imu; //gyro
    DcMotor backLeft, backRight, frontLeft, frontRight;
    LinearSlide slide;
    CarouselRotator rotator;
    private MecanumDrive drive;
    private RevColorSensorV3 color;

    private float cumulativeAngle = 0;
    private float prevAngle = 0;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private List<Recognition> objectRecognitions;


    class TelemetryData implements Runnable {


        @Override
        public void run() {
            telemetry.addData("X", imu.getPosition().x);
            telemetry.addData("Y", imu.getPosition().y);
            telemetry.addData("Z", imu.getPosition().z);
        }
    }

    @Override
    public void runOpMode() {

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        backLeft = hardwareMap.dcMotor.get("backleft");
        backRight = hardwareMap.dcMotor.get("backright");
        frontLeft = hardwareMap.dcMotor.get("frontleft");
        frontRight = hardwareMap.dcMotor.get("frontright");
        slide = new LinearSlide(hardwareMap.get(DcMotor.class, "linearslide"), hardwareMap.get(DcMotor.class, "rotator"), hardwareMap.get(Servo.class, "grabber"));
        rotator = new CarouselRotator(hardwareMap.get(DcMotor.class, "carousel"));
        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight, imu);
        color = hardwareMap.get(RevColorSensorV3.class, "color");
        telemetry.addData(">", "Initializing");
        telemetry.update();
        initCV();
        while (!imu.isSystemCalibrated() && opModeIsActive()) {
            wait(50);
        }
        imu.startAccelerationIntegration(new Position(DistanceUnit.CM, 0, 0, 0, 0), new Velocity(DistanceUnit.CM, 0, 0, 0, 0), 100);
        telemetry.addData(">", "Ready");
        telemetry.update();
        waitForStart();
        if (!opModeIsActive()) {
            return;
        }
        telemetry.addAction(new TelemetryData());
        Initializer.initializeGrabber(slide.grabber, slide.slide, this);
        slide.setGrabberPosition(true, false);
        DuckPosition position = searchForDuck();
        telemetry.addData("Status", "Duck found at " + position.toString());
        telemetry.update();
        strafe(true, 1, 675, 0, true);
        waitOnSlidePosition(position == DuckPosition.LEFT ? teleOp.heightPresets[1] : (position == DuckPosition.MIDDLE ? teleOp.heightPresets[2] : teleOp.heightPresets[3]), position == DuckPosition.RIGHT ? 300 : 0);
        forward(0.6, 1175, 0);
        slide.setGrabberPosition(false, true);


        forward(-0.6, 800, 0);
        slide.goDown();

        turn(-90);
        forward(0.75, 10000, -90, 9.5);
        strafe(false, 0.2, 2000, -90, true);
        rotator.setRotatorPower(0.8);
        wait(3000);
        rotator.setRotatorPower(0);
        turn(-90);
        strafe(true, 1, 350, -90, true);
        slide.grabber.setPosition(0.6);
        slide.goToPosition(2250);
        forward(-1, 500, -90);
        wait(100);
        turn(90);
        wait(100);
        strafe(false, 1, 245, 90, true);
        wait(100);
        forward(1, 4000, 90, 8);
        forward(-0.45, 500, 90);
        slide.goDown();
        slide.setGrabberPosition(false, true);

        wait(5000);
    }

    private DuckPosition searchForDuck() {
        /*
        find the duck!
        dependent on webcam alignment with center barcode
        Process:
        1. Check if the duck is in the center position
        2. If not found, check the right position
        3. If not found, assume the duck is on the left
         */

        forward(0.2, 250, 0);
        telemetry.addData("Status", "Searching for Center Duck");
        telemetry.addData("Orientation", cumulativeAngle);
        telemetry.update();
        Recognition duck = waitUntilObjectFound("Duck", 2000);
        strafe(true, 1, 500, 0, true);
        if (duck != null) {
            return DuckPosition.MIDDLE;
        }
        telemetry.addData("Status", "Searching for Right Duck");
        telemetry.update();
        duck = waitUntilObjectFound("Duck", 2000);
        return duck != null ? DuckPosition.RIGHT : DuckPosition.LEFT;
    }


    private void waitOnSlidePosition(int lift, int rotate) {
        /*
        Synchronously rotates the slide to the desired position
         */
        slide.goToPosition(lift);
        slide.setRotation(rotate);

        // stays in loop while the slide is approaching the target position
        while ((Math.abs(lift - slide.slide.getCurrentPosition()) > 50 || Math.abs(rotate - slide.rotator.getCurrentPosition()) > 50) && opModeIsActive()) {
            telemetry.addData("Slide", slide.slide.getCurrentPosition());
            telemetry.addData("Slide Target", lift);
            telemetry.addData("Rotator", slide.rotator.getCurrentPosition());
            telemetry.addData("Rotator Target", rotate);
            telemetry.update();
        }
        slide.slide.setPower(0);
        slide.rotator.setPower(0);
    }

    private double[] getColorData() {
        /*
        Get the color data from the RevV3 Color Sensor, applying softmax to rgb
         */
        int red = color.red();
        int green = color.green();
        int blue = color.blue();
        int total = red + green + blue;
        return new double[]{
                (double) red / total,
                (double) green / total,
                (double) blue / total,
                color.alpha(),
                color.getDistance(DistanceUnit.CM)
        };
    }


    private void showOrientation() {
        telemetry.addData("Orientation", cumulativeAngle);
    }

    private void turn(double desiredAngle) {
        turn(desiredAngle, 0.8, 0.4, 0.8);
    }

    private void turn(double desiredAngle, double turnPower, double closeRangePower, double slowDownCutoff) {
        // gyroscopic turning for autonomous
        updateOrientation();
        float initialAngle = cumulativeAngle;

        // determines the direction needed to turn
        boolean right = desiredAngle > cumulativeAngle;

        float turningMultiplier = right ? -1 : 1;

        while (opModeIsActive()) {
            updateOrientation();
            /*
            Decreases turning power as the bot gets closer to the desired angle
             */
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


    private void forward(double power, int maxMillis, float maintainAngle) {
        forward(power, maxMillis, maintainAngle, Integer.MIN_VALUE);
    }

    private void forward(double power, int maxMillis, float maintainAngle, double distanceThreshold) {
        long endTime = SystemClock.elapsedRealtime() + maxMillis;
        while (opModeIsActive()) {
            updateOrientation();
            // determines how far off from desired angle the robot is, and computes adjustment factor for motors
            double error = maintainAngle - cumulativeAngle;
            double offset = power * error * 0.1;

            showOrientation();
            telemetry.addData("Power Offset", offset);
            telemetry.addData("Time Remaining", endTime - SystemClock.elapsedRealtime());
            telemetry.update();

            drive.setMotorPowers(power + offset, power - offset, power, power);

            // checks that the max time hasn't elapsed and that the distance measurement is above the threshold
            if (SystemClock.elapsedRealtime() >= endTime || getColorData()[4] < distanceThreshold) {
                break;
            }
        }
        drive.setMotorPowers(0, 0, 0, 0);
    }

    private void strafe(boolean right, double power, int millis, float maintainAngle, boolean useCorrection) {
        // gyroscopically-assisted strafing
        long endTime = SystemClock.elapsedRealtime() + millis;
        int mult = right ? 1 : -1;
        while (opModeIsActive()) {
            updateOrientation();
            double offset;
            if (useCorrection) {
                double error = maintainAngle - cumulativeAngle;
                offset = power * error * 0.05;
            } else {
                offset = 0;
            }
            showOrientation();
            telemetry.addData("Power Offset", offset);
            telemetry.addData("Time Remaining", endTime - SystemClock.elapsedRealtime());
            telemetry.update();

            drive.setMotorPowers(power * mult + offset, -power * mult - offset, -power * mult + offset, power * mult - offset);

            if (SystemClock.elapsedRealtime() >= endTime) {
                break;
            }
        }
        drive.setMotorPowers(0, 0, 0, 0);
    }

    private void wait(int millis) {
        /*
        Waits for the specified amount of time
        Can't use Thread.sleep because the robotcontroller freaks out
        Horrifically inefficient but it gets the job done
         */
        long initTime = SystemClock.elapsedRealtime();
        long endTime = initTime + millis;
        while (opModeIsActive()) {
            if (SystemClock.elapsedRealtime() > endTime) {
                break;
            }
        }

    }

    private Recognition waitUntilObjectFound(String label, long msTimeout) {
        /*
        Waits for the tflite model to detect an object of the specified type
        If not found within the time frame, returns null
         */
        long end = SystemClock.elapsedRealtime() + msTimeout;
        while (opModeIsActive()) {
            updateRecognitions();
            if (objectRecognitions != null) {
                Recognition recognition = recognitionsContainsLabel(objectRecognitions, label);
                if (recognition != null) {
                    return recognition;
                }
            }
            if (SystemClock.elapsedRealtime() >= end) {
                break;
            }
        }
        return null;
    }

    private Recognition recognitionsContainsLabel(List<Recognition> recognitions, String label) {
        for (Recognition recognition : recognitions) {
            if (recognition.getLabel().equals(label)) {
                return recognition;
            }
        }
        return null;
    }

    private void updateOrientation() {
        /*
        Acts as an accumulation function for the imu
        Rev IMU outputs position as a discrete range from -180 to 180
        This gets mapped to 0 to 360, which can then be used to compute the overall cumulative angle
         */
        float currentAngle = processAngle(drive.getOrientation().thirdAngle);
        telemetry.addData("processedangle", currentAngle);
        if (Math.abs(prevAngle - currentAngle) > 300) {  //checks if angle has wrapped around
            if (currentAngle < 180) { // crossed from 360 to 0
                //this.cumulativeAngle += (360 - prevAngle) + currentAngle;
                this.cumulativeAngle += 360 + currentAngle - prevAngle;
            } else { // crossed from 0 to 360
                //this.cumulativeAngle -= (360 - currentAngle) + prevAngle;
                this.cumulativeAngle += currentAngle - prevAngle - 360;
            }
        } else {
            this.cumulativeAngle += currentAngle - prevAngle;
        }
        this.prevAngle = currentAngle;
    }


    private float processAngle(float angle) {
        // maps angle from -180 to 180 to 0 to 360
        angle = angle * -1;
        return (angle < 0) ? angle + 360 : angle;
    }

    // from https://stackoverflow.com/questions/7505991/arduino-map-equivalent-function-in-java
    double mapValue(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    private void initCV() {
        initCV(1.35, 16.0 / 9.0);
    }

    private void initCV(double magnification, double aspectRatio) {
        /*
        Initializes computer vision for duck detection
        Stolen directly from the FTCRobotController sample programs
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam"); // configures usb2 webcam

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.40f; // will only output result if more than 70% sure
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TENSORFLOW_ASSET_NAME, LABELS); // loads model

        // check if model was loaded successfully
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(magnification, aspectRatio);
        }
    }

    private void updateRecognitions() {
        // updates list of tfod recognitions
        if (tfod != null) {
            List<Recognition> currentRecognitions = tfod.getUpdatedRecognitions();
            if (currentRecognitions != null) {
                objectRecognitions = currentRecognitions;
            }
        }
    }

    private void displayRecognitions(List<Recognition> recognitions) {
        int i = 0;
        if (recognitions != null) {
            for (Recognition recognition : recognitions) {
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




