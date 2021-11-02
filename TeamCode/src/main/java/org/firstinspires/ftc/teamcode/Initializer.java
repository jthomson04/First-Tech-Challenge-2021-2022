package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Initializer {
    public static void initializeGrabber(Servo grabber, DcMotor linearSlide, LinearOpMode opMode) {
        grabber.setPosition(0.6);
        while (grabber.getPosition() < 0.6) ;
        opMode.telemetry.addData("done servo", "true");
        opMode.telemetry.update();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        linearSlide.setTargetPosition(240);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(-1);
        while (linearSlide.getCurrentPosition() < 235) ;
        linearSlide.setPower(0);
        opMode.telemetry.addData("done lifting", "true");
        opMode.telemetry.update();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
