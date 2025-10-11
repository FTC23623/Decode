package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import org.firstinspires.ftc.teamcode.types.IntakeActions;

@Autonomous(name = "HydrAuto1", preselectTeleOp = "HyDrive_Red")
public class HydrAuto1 extends HydrAuto {

    public HydrAuto1() {
        mBeginPose = new Pose2d(60, 15, Math.toRadians(0));
    }

    @Override
    protected SequentialAction CreateAuto() {

        Pose2d Launch=new Pose2d(55, 15, Math.toRadians(-20));
        Pose2d GPP_WP=new Pose2d(35, 30,Math.toRadians(90));
        Pose2d GPP=new Pose2d(35,48,Math.toRadians(90));
        Pose2d PGP_WP=new Pose2d(11, 30,Math.toRadians(90));
        Pose2d PGP=new Pose2d(11,48,Math.toRadians(90));
        Pose2d PPG_WP=new Pose2d(-12, 30,Math.toRadians(90));
        Pose2d PPG=new Pose2d(-12,48,Math.toRadians(90));

        Action driveToLaunch1 = mDrive.actionBuilder(mBeginPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(Launch, Math.toRadians(180))
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(GPP_WP, Math.toRadians(135))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(GPP, Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(Launch, Math.toRadians(0))
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(PGP_WP, Math.toRadians(135))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(PGP, Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(Launch, Math.toRadians(0))
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(PPG_WP, Math.toRadians(135))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(PPG, Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(Launch, Math.toRadians(0))
                .waitSeconds(1.5)
                .build();

        return new SequentialAction(driveToLaunch1);
    }
}
