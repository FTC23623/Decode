package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Config
@TeleOp(name = "LinearLaunch")
public class LinearLaunch extends LinearOpMode {

    private DcMotor left;
    private DcMotor right;
    public static double power=0.0;
    @Override
    public void runOpMode() throws InterruptedException
    {
        left = hardwareMap.get(DcMotor.class, "left");
        right= hardwareMap.get(DcMotor.class, "right");
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            left.setPower(power);
            right.setPower(power);
            idle();
        }
    }
}

