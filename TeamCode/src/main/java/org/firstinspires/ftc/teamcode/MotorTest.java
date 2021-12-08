package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Motor Test")
public class MotorTest extends LinearOpMode {
    public static final int[] heightPresets = {0, 1300, 3050, 5500, 7500};
    private MecanumDrive drive;
    private LinearSlide slide;
    private CarouselRotator rotator;
    private final int currentTarget = 0;


    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backright");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight, imu);

        slide = new LinearSlide(hardwareMap.get(DcMotor.class, "linearslide"), hardwareMap.get(DcMotor.class, "rotator"), hardwareMap.get(Servo.class, "grabber"));

        rotator = new CarouselRotator(hardwareMap.get(DcMotor.class, "carousel"));
        waitForStart();
        while (opModeIsActive()) {
            drive.setMotorPowers(1, 1, 1, 1);
        }

    }


    double toExp2(double num) {
        return Math.pow(num, 2) * (num > 0 ? 1 : -1);
    }
}
