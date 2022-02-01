package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;

public class CarouselRotator {
    DcMotor rotator;

    public CarouselRotator(DcMotor rotator) {
        this.rotator = rotator;
        this.rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setRotatorPower(double power) {
        rotator.setPower(power);
    }
}
