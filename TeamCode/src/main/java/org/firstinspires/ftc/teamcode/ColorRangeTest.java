package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ColorRangeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 color = hardwareMap.get(RevColorSensorV3.class, "color");
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("Alpha", color.alpha());
            telemetry.addData("Proximity", color.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
