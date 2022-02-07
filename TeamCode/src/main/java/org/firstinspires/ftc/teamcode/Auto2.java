package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
public class Auto2 extends LinearOpMode {

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

    private List<Recognition> objectRecognitions; // a list of recognitions returned by the tflite model

    // A telemetry action to display xyz position on every telemetry.update() call

    class TelemetryData implements Runnable {
        @Override
        public void run() {

            telemetry.addData("X", imu.getPosition().x);
            telemetry.addData("Y", imu.getPosition().y);
            telemetry.addData("Z", imu.getPosition().z);
            telemetry.addData("Orientation", cumulativeAngle);
        }
    }

    @Override
    public void runOpMode() {


        // connect to drive motors
        backLeft = hardwareMap.dcMotor.get("backleft");
        backRight = hardwareMap.dcMotor.get("backright");
        frontLeft = hardwareMap.dcMotor.get("frontleft");
        frontRight = hardwareMap.dcMotor.get("frontright");

        // connect to peripheral motors and sensors
        slide = new LinearSlide(hardwareMap.get(DcMotor.class, "linearslide"), hardwareMap.get(DcMotor.class, "rotator"), hardwareMap.get(Servo.class, "grabber"));
        rotator = new CarouselRotator(hardwareMap.get(DcMotor.class, "carousel"));
        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight, imu);
        color = hardwareMap.get(RevColorSensorV3.class, "color");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        telemetry.addData(">", "Initializing");
        telemetry.update();

        // Ensure the IMU has completed calibrating

        while (!imu.isSystemCalibrated() && opModeIsActive()) {
            wait(50);
        }

        // initialize position detection
        imu.startAccelerationIntegration(new Position(DistanceUnit.CM, 0, 0, 0, 0), new Velocity(DistanceUnit.CM, 0, 0, 0, 0), 20);


        telemetry.addData(">", "Ready");
        telemetry.update();

        // wait for run button to be pressed
        waitForStart();
        // prevents it from spasming randomly when abortion on initialization menu
        if (!opModeIsActive()) {
            return;
        }

        // add the XYZ positioning data to telemetry output
        telemetry.addAction(new TelemetryData());

        // deploy grabber and lift it to allow distance sensor to detect capstone
        Initializer.initializeGrabber(slide.grabber, slide.slide, this);
        slide.setGrabberPosition(true, false);
        slide.goToPosition(teleOp.heightPresets[2]);

        DuckPosition position = searchForDuck();

        // shift to align with shipping hub
        strafe(true, 1, 675, 0, true);

        waitOnSlidePosition(position == DuckPosition.LEFT ? teleOp.heightPresets[1] : (position == DuckPosition.MIDDLE ? teleOp.heightPresets[2] : teleOp.heightPresets[3]), position == DuckPosition.RIGHT ? 300 : 0);
        forward(0.6, 1175, 0);
        slide.setGrabberPosition(false, true);
        forward(-0.6, 800, 0);
        slide.goDown();

        // head to carousel
        turn(-90);
        forward(0.75, 10000, -90, 9.5);
        strafe(false, 0.2, 2000, -90, true);
        rotator.setRotatorPower(0.8);
        wait(3000);
        rotator.setRotatorPower(0);

        // head to warehouse
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

        forward(0.25, 1500, 0);
        DuckPosition position = null;
        if (getColorData()[4] < 5) {
            position = DuckPosition.MIDDLE;
        }
        strafe(true, 0.3, 1000, 0, true);

        // check if the duck hasn't already been found
        if (position == null) {
            if (getColorData()[4] < 5) {
                position = DuckPosition.RIGHT;
            } else {
                position = DuckPosition.LEFT;
            }
        }
        return position;

    }


    private void waitOnSlidePosition(int lift, int rotate) {
        /*
        Synchronously rotates the slide to the desired position
         */
        slide.goToPosition(lift);
        slide.setRotation(rotate);

        // stays in loop while the slide and rotator are approaching the target position
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

    private void turn(double desiredAngle) {
        turn(desiredAngle, 0.8, 0.4, 0.8);
    }

    private void turn(double desiredAngle, double turnPower, double closeRangePower, double slowDownCutoff) {

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
}




