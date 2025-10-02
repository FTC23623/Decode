package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
@TeleOp(name = "limelight")
public class limelight extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();
        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                    
                    double targetOffsetAngle_Vertical = result.getTy();

                    // how many degrees back is your limelight rotated from perfectly vertical?
                    double limelightMountAngleDegrees = -1.34;

                    // distance from the center of the Limelight lens to the floor
                    double limelightLensHeightInches = 4.5;

                    // distance from the target to the floor
                    double goalHeightInches = 9.5;

                    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

                    //calculate distance
                    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
                    telemetry.addData("GoalDistance", distanceFromLimelightToGoalInches);
                    telemetry.update();
                }
            }
        }
    }
}

