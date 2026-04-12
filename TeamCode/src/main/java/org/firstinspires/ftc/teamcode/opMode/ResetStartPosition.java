package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.objects.OpmodeHeading;

@TeleOp
public class ResetStartPosition extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpmodeHeading.SetOffset(null);
        telemetry.addLine("Start position reset");
        telemetry.update();
        waitForStart();
    }
}
