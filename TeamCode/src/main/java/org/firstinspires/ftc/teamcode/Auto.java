package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends LinearOpMode {

    private static final String TENSORFLOW_ASSET_NAME = "FreightFrenzy_BCDM.tflite"; // name of presaved bundle tflite model
    private static final String[] LABELS = { // labels for tflite model
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY = "AcRDh/L/////AAABmSXgpqzx70p0vDh2eqv9N6YEI6fwUF4tbXNAvrYXcMt+Zbq6qXM2z4Aq7KnWGblN2ZiCjGIpLlWeXL7IQrtvBJFzPNil285nzLsLg/KWynX1Pss7RKL7i/O4hFxsorVD/+4kMvkFMV7q1uVt4mY4d+SuChH3vAQA6t5NnVJGhh6M+eAeLcQYTF9KCkNL0xgXYeg06BPbppTydDgNRqTrsGwZgegIHutSHH89R/P1NdR9arRifjrfUtNEoIHglPMJ7Mh3PeFH3CpcTBfdgCuYcQCZb0lGAMI8v0Nlwh6lHkRmUFjQsfR+ujiiAAx0agouc2mEy1dK/lLDq34ZtcoAqNEpI1zinV8lkpVvFE3y9xL4";
    private AutoDrive autoDrive;

    DcMotor backLeft, backRight, frontLeft, frontRight;
    LinearSlide slide;
    CarouselRotator rotator;
    private MecanumDrive drive;
    private RevColorSensorV3 color;


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private List<Recognition> objectRecognitions;




    @Override
    public void runOpMode() {

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        backLeft = hardwareMap.dcMotor.get("backleft");
        backRight = hardwareMap.dcMotor.get("backright");
        frontLeft = hardwareMap.dcMotor.get("frontleft");
        frontRight = hardwareMap.dcMotor.get("frontright");
        color = hardwareMap.get(RevColorSensorV3.class, "color");
        autoDrive = new AutoDrive(new MecanumDrive(frontLeft, frontRight, backLeft, backRight, imu), color, this);

        slide = new LinearSlide(hardwareMap.get(DcMotor.class, "linearslide"), hardwareMap.get(DcMotor.class, "rotator"), hardwareMap.get(Servo.class, "grabber"));
        rotator = new CarouselRotator(hardwareMap.get(DcMotor.class, "carousel"));


        initCV();
        waitForStart();
        Initializer.initializeGrabber(slide.grabber, slide.slide, this);
        slide.setGrabberPosition(true, false);
        DuckPosition position = searchForDuck();
        telemetry.addData("Status", "Duck found at " + position.toString());
        telemetry.update();
        autoDrive.strafe(true, 1, 640, 0);
        waitOnSlidePosition(position == DuckPosition.LEFT ? teleOp.heightPresets[1] : (position == DuckPosition.MIDDLE ? teleOp.heightPresets[2] : teleOp.heightPresets[3]), position == DuckPosition.RIGHT ? 300 : 0);
        autoDrive.forward(0.6, 1175, 0);
        slide.setGrabberPosition(false, true);


        autoDrive.forward(-0.6, 800, 0);
        slide.goDown();

        autoDrive.turn(-90);
        autoDrive.forward(0.75, 10000, -90, 8);
        autoDrive.forward(-0.1, 750, -90);
        autoDrive.strafe(false, 0.35, 900, -90);
        rotator.setRotatorPower(0.8);
        wait(3000);
        rotator.setRotatorPower(0);
        autoDrive.strafe(true, 1, 350, -90);
        slide.grabber.setPosition(0.6);
        slide.goToPosition(1500);
        autoDrive.forward(-1, 500, -90);
        autoDrive.turn(90);
        autoDrive.strafe(false, 1, 350, 90);
        autoDrive.forward(1, 4500, 90, 5);
        autoDrive.forward(-0.2, 1000, 90);
        slide.setGrabberPosition(false, true);
        slide.goDown();
        wait(2000);
        /*strafe(true, 0.3, 2600, -90);
        forward(0.28, 10000, -90, 5);
        wait(20000);
        rotator.setRotatorPower(0.2);
        strafe(true, 0.5, 1000, -90);*/
    }

    private DuckPosition searchForDuck() {
        /*
        find the duck!
        relies on webcam alignment with center barcode
        Process:
        1. Check if the duck is in the center position
        2. If not found, check the right position
        3. If not found, assume the duck is on the left
         */

        autoDrive.forward(0.2, 250, 0);
        telemetry.addData("Status", "Searching for Center Duck");
        telemetry.update();
        Recognition duck = waitUntilObjectFound("Duck", 2000);
        autoDrive.strafe(true, 1, 635, 0);
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








    private void initCV() {
        initCV(1.3, 16.0 / 9.0);
    }

    private void initCV(double magnification, double aspectRatio) {
        /*
        Initializes computer vision for duck detection
         */
        telemetry.addData("CV", "Initializing");
        telemetry.update();
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam"); // configures usb2 webcam

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.70f; // will only output result if more than 70% sure
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TENSORFLOW_ASSET_NAME, LABELS); // loads model

        // check if model was loaded successfully
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(magnification, aspectRatio);
        }
        telemetry.addData("CV", "Activated");
        telemetry.update();
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




