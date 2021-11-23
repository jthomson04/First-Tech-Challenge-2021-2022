package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TeleOp")
public class teleOp extends LinearOpMode {
    private static final int[] heightPresets = {0, 940, 2520, 5400, 7500};
    private MecanumDrive drive;
    private LinearSlide slide;
    private CarouselRotator rotator;
    private int currentTarget = 0;


    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backright");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        int presses = 0;
        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight, imu);

        slide = new LinearSlide(hardwareMap.get(DcMotor.class, "linearslide"), hardwareMap.get(DcMotor.class, "rotator"), hardwareMap.get(Servo.class, "grabber"));

        rotator = new CarouselRotator(hardwareMap.get(DcMotor.class, "carousel"));


        double forward;
        double strafe;
        double rotate;
        double modifier;

        boolean targetMode = false;
        boolean prevTargetMode;
        boolean previouslyUpDownPressed = false;

        waitForStart();

        long prevPress = 0;
        while (opModeIsActive()) {
            prevTargetMode = targetMode;
            targetMode = gamepad2.dpad_up || gamepad2.dpad_down || prevTargetMode;
            modifier = 1 - 0.8 * gamepad1.right_trigger;

            forward = toExp2(-gamepad1.left_stick_y);
            strafe = toExp2(gamepad1.left_stick_x);
            rotate = toExp2(gamepad1.right_stick_x);

            double frontLeftPow = (forward + strafe + rotate) * modifier;
            double backLeftPow = (forward - strafe + rotate) * modifier;
            double frontRightPow = (forward - strafe - rotate) * modifier;
            double backRightPow = (forward + strafe - rotate) * modifier;

            drive.setMotorPowers(frontLeftPow, frontRightPow, backLeftPow, backRightPow);
            if (gamepad2.left_stick_y != 0) { // regular rotation mode
                targetMode = false;
                slide.setSlidePower(-gamepad2.left_stick_y);
                telemetry.addData("Slide Mode", "Normal");
            } else if (gamepad2.a) {
                targetMode = false;
                slide.goDown();
                telemetry.addData("Slide Mode", "Down");
                currentTarget = 0;
            } else if (targetMode) {
                if ((gamepad2.dpad_up || gamepad2.dpad_down) && !previouslyUpDownPressed && System.currentTimeMillis() - prevPress > 200) {
                    previouslyUpDownPressed = true;
                    currentTarget = Math.max(currentTarget + (gamepad2.dpad_up ? 1 : -1), 0) % heightPresets.length;
                    presses += 1;
                    prevPress = System.currentTimeMillis();
                } else {
                    previouslyUpDownPressed = false;
                }
                slide.goToPosition(heightPresets[currentTarget]);
                telemetry.addData("Slide Mode", "Run to Position");
                telemetry.addData("PreviousPress", previouslyUpDownPressed);
                telemetry.addData("Current Target", heightPresets[currentTarget]);
                telemetry.addData("presses", presses);
            } else {
                slide.setSlidePower(-gamepad2.left_stick_y);
            }
            if (!gamepad2.a) {
                slide.setRotatorPower(-gamepad2.right_stick_y * .25);
            }


            rotator.setRotatorPower(-gamepad2.right_trigger + gamepad2.left_trigger);
            slide.setGrabberPosition(gamepad2.b, gamepad2.x);


            telemetry.addData("Drive", forward);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotate", rotate);

            Orientation orientation = drive.getOrientation();
            telemetry.addData("X", orientation.firstAngle);
            telemetry.addData("Y", orientation.secondAngle);
            telemetry.addData("Z", orientation.thirdAngle);

            telemetry.addData("Linear Slide Position", slide.slide.getCurrentPosition());
            telemetry.addData("Rotator Position", slide.rotator.getCurrentPosition());
            telemetry.addData("Grabber Position", slide.grabber.getPosition());
            telemetry.update();


        }
    }

    double toExp2(double num) {
        int mult = 1;
        if (num < 0) {
            mult = -1;
        }
        return Math.pow(num, 2) * mult;
    }
}
