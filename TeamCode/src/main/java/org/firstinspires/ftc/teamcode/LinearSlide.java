package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class LinearSlide {
    DcMotor slide;
    DcMotor rotator;
    Servo grabber;

    public LinearSlide(DcMotor slide, DcMotor rotator, Servo grabber) {
        this.slide = slide;
        this.slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.slide.setDirection(DcMotorSimple.Direction.REVERSE);
        this.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.rotator = rotator;
        this.rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.grabber = grabber;
    }

    public void setRotatorPower(double rotatorPower) {
        rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (rotatorPower <= 0 && rotator.getCurrentPosition() < 50) {
            rotator.setPower(0);
        } else {
            rotator.setPower(rotatorPower);
        }
    }

    public void setSlidePower(double slidePower) {

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (slidePower <= 0 && slide.getCurrentPosition() < 50) {
            slide.setPower(0);
        } else {
            slide.setPower(slidePower);
        }
    }

    public int getPosition() {
        return slide.getCurrentPosition();
    }

    public void goDown() {
        rotator.setTargetPosition(0);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
        rotator.setPower(0.35);
    }

    public void setGrabberPosition(boolean close, boolean open) {
        if (open || close) {
            grabber.setPosition(close ? 1 : 0.5);
        }
    }

    public void goToPosition(int position) {
        slide.setTargetPosition(position);
        slide.setPower(slide.getCurrentPosition() > position ? -1 : 1);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
