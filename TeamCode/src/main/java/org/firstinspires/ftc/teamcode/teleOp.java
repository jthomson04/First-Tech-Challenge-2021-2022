package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TeleOp")
public class teleOp extends LinearOpMode {
    public static final int[] heightPresets = {0, 1300, 3050, 5500, 7500};
    private int currentTarget = 0;


    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backright");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        int dpadPresses = 0;

        MecanumDrive drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight, imu);

        LinearSlide slide = new LinearSlide(hardwareMap.get(DcMotor.class, "linearslide"), hardwareMap.get(DcMotor.class, "rotator"), hardwareMap.get(Servo.class, "grabber"));

        CarouselRotator rotator = new CarouselRotator(hardwareMap.get(DcMotor.class, "carousel"));

        boolean targetMode = false;
        boolean prevTargetMode;
        boolean previouslyUpDownPressed = false;



        waitForStart();

        long prevPress = 0;
        while (opModeIsActive()) {
            drive.control(gamepad1, telemetry);

            prevTargetMode = targetMode;
            targetMode = gamepad2.dpad_up || gamepad2.dpad_down || prevTargetMode;

            if (gamepad2.left_stick_y != 0) { // regular rotation mode
                targetMode = false;
                slide.setSlidePower(-gamepad2.left_stick_y * (1 - 0.8 * gamepad2.right_trigger));
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
                    dpadPresses += 1;
                    prevPress = System.currentTimeMillis();
                } else {
                    previouslyUpDownPressed = false;
                }
                slide.goToPosition(heightPresets[currentTarget]);
                telemetry.addData("Slide Mode", "Run to Position");
                telemetry.addData("PreviousPress", previouslyUpDownPressed);
                telemetry.addData("Current Target", heightPresets[currentTarget]);
                telemetry.addData("dpadPresses", dpadPresses);
            } else {
                slide.setSlidePower(-gamepad2.left_stick_y * (1 - 0.8 * gamepad2.right_trigger));
            }
            if (!gamepad2.a) {
                slide.setRotatorPower(-gamepad2.right_stick_y * .25 * (1 - 0.8 * gamepad2.right_trigger));
            }


            rotator.setRotatorPower((gamepad2.right_bumper ? -1 : (gamepad2.left_bumper ? 1 : 0)) * (1 - 0.8 * gamepad2.right_trigger));
            slide.setGrabberPosition(gamepad2.b, gamepad2.x);

            telemetry.addData("Linear Slide Position", slide.slide.getCurrentPosition());
            telemetry.addData("Rotator Position", slide.rotator.getCurrentPosition());
            telemetry.addData("Grabber Position", slide.grabber.getPosition());
            telemetry.update();


        }
    }


}
