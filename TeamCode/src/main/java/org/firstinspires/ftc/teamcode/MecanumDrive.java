package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MecanumDrive {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final BNO055IMU imu;

    public MecanumDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, BNO055IMU imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.imu = imu;

        BNO055IMU.Parameters gyroParams = new BNO055IMU.Parameters();
        gyroParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParams.calibrationDataFile = "BNO055IMUCalibration";
        this.imu.initialize(gyroParams);

        DcMotor[] motors = {this.frontLeft, this.frontRight, this.backLeft, this.backRight};

        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        this.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Orientation getOrientation() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }

    public void setMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        this.frontLeft.setPower(frontLeft);
        this.frontRight.setPower(frontRight);
        this.backLeft.setPower(backLeft);
        this.backRight.setPower(backRight);
    }

}
