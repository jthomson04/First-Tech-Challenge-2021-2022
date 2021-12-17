package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoDrive {
    private MecanumDrive drive;
    private LinearOpMode opMode;
    private BNO055IMU imu;
    RevColorSensorV3 color;
    public float cumulativeAngle = 0;
    private float prevAngle = 0;

    public AutoDrive(MecanumDrive drive, RevColorSensorV3 color, LinearOpMode opMode) {
        this.drive = drive;
        this.opMode = opMode;
        this.color = color;
    }

    public void forward(double power, int maxMillis, float maintainAngle) {
        forward(power, maxMillis, maintainAngle, Integer.MIN_VALUE);
    }

    public void forward(double power, int maxMillis, float maintainAngle, double distanceThreshold) {
        long endTime = SystemClock.elapsedRealtime() + maxMillis;

        while (opMode.opModeIsActive()) {
            updateOrientation();
            // determines how far off from desired angle the robot is, and computes adjustment factor for motors
            double error = maintainAngle - cumulativeAngle;
            double offset = power * error * 0.1;

            showOrientation();
            opMode.telemetry.addData("Power Offset", offset);
            opMode.telemetry.addData("Time Remaining", endTime - SystemClock.elapsedRealtime());
            opMode.telemetry.update();

            drive.setMotorPowers(power + offset, power - offset, power, power);

            // checks that the max time hasn't elapsed and that the distance measurement is above the threshold
            if (SystemClock.elapsedRealtime() >= endTime || getColorData()[4] < distanceThreshold) {
                break;
            }
        }
        drive.setMotorPowers(0, 0, 0, 0);
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

    public void turn(double desiredAngle) {
        turn(desiredAngle, 0.8, 0.4, 0.8);
    }

    public void turn(double desiredAngle, double turnPower, double closeRangePower, double slowDownCutoff) {
        // gyroscopic turning for autonomous
        updateOrientation();
        float initialAngle = cumulativeAngle;

        // determines the direction needed to turn
        boolean right = desiredAngle > cumulativeAngle;

        float turningMultiplier = right ? -1 : 1;

        while (opMode.opModeIsActive()) {
            updateOrientation();
            /*
            Decreases turning power as the bot gets closer to the desired angle
             */
            double percentageThrough = mapValue(cumulativeAngle, initialAngle, desiredAngle, 0, 1);
            double power = percentageThrough > slowDownCutoff ? closeRangePower : turnPower;

            showOrientation();
            opMode.telemetry.addData("Power", power);
            opMode.telemetry.addData("Percentage Through Turn", percentageThrough);
            opMode.telemetry.update();

            drive.setMotorPowers(power * -1 * turningMultiplier, power * turningMultiplier, power * -1 * turningMultiplier, power * turningMultiplier);
            if (Math.abs(desiredAngle - cumulativeAngle) < 2) {
                break;
            }
        }
        drive.setMotorPowers(0, 0, 0, 0);
    }

    public void strafe(boolean right, double power, int millis, float maintainAngle) {
        // gyroscopically-assisted strafing
        long endTime = SystemClock.elapsedRealtime() + millis;
        int mult = right ? 1 : -1;
        while (opMode.opModeIsActive()) {
            updateOrientation();
            double error = maintainAngle - cumulativeAngle;
            double offset = power * error * 0.1;
            showOrientation();
            opMode.telemetry.addData("Power Offset", offset);
            opMode.telemetry.addData("Time Remaining", endTime - SystemClock.elapsedRealtime());
            opMode.telemetry.update();

            drive.setMotorPowers(power * mult + offset, -power * mult - offset, -power * mult + offset, power * mult - offset);

            if (SystemClock.elapsedRealtime() >= endTime) {
                break;
            }
        }
        drive.setMotorPowers(0, 0, 0, 0);
    }


    private void showOrientation() {
        opMode.telemetry.addData("Orientation", cumulativeAngle);
    }

    private void updateOrientation() {
        /*
        Acts as an accumulation function for the imu
        Rev IMU outputs position as a discrete range from -180 to 180
        This gets mapped to 0 to 360, which can then be used to compute the overall cumulative angle
         */
        float currentAngle = processAngle(drive.getOrientation().thirdAngle);
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
    private double mapValue(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

}
