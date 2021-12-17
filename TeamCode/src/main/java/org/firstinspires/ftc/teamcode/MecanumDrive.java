package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    private double toExp2(double num) {
        return Math.pow(num, 2) * (num > 0 ? 1 : -1);
    }

    public void control(Gamepad gamepad, Telemetry telemetry) {
        double modifier = 1 - 0.8 * gamepad.right_trigger;

        double forward = toExp2(-gamepad.left_stick_y);
        double strafe = toExp2(gamepad.left_stick_x);
        double rotate = toExp2(gamepad.right_stick_x);

        double frontLeftPow = (forward + strafe + rotate) * modifier;
        double backLeftPow = (forward - strafe + rotate) * modifier;
        double frontRightPow = (forward - strafe - rotate) * modifier;
        double backRightPow = (forward + strafe - rotate) * modifier;

        setMotorPowers(frontLeftPow, frontRightPow, backLeftPow, backRightPow);
        telemetry.addData("Drive", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Rotate", rotate);

        Orientation orientation = getOrientation();
        telemetry.addData("X", orientation.firstAngle);
        telemetry.addData("Y", orientation.secondAngle);
        telemetry.addData("Z", orientation.thirdAngle);

    }

}
