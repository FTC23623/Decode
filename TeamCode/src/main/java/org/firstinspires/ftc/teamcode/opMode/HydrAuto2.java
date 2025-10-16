package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "HydrAuto2", preselectTeleOp = "HyDrive_Red")
public class HydrAuto2 extends HydrAuto {

    public HydrAuto2() {
        mBeginPose = new Pose2d(-54, 52, Math.toRadians(310));
    }

    @Override
    protected SequentialAction CreateAuto() {

        Pose2d GPP_WP=new Pose2d(46, 30,Math.toRadians(90));
        Pose2d GPP=new Pose2d(46,54,Math.toRadians(90));
        Pose2d PGP_WP=new Pose2d(22, 30,Math.toRadians(90));
        Pose2d PGP=new Pose2d(22,54,Math.toRadians(90));
        Pose2d PPG_WP=new Pose2d(-4, 30,Math.toRadians(90));
        Pose2d PPG=new Pose2d(-4,54,Math.toRadians(90));
        Pose2d LaunchNear=new Pose2d(-25, 24, Math.toRadians(-40));

        Action driveToLaunch1 = mDrive.actionBuilder(mBeginPose)
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(LaunchNear, Math.toRadians(315))
                .waitSeconds(1.5)
                .splineToLinearHeading(PPG_WP, Math.toRadians(90))
                .splineToLinearHeading(PPG, Math.toRadians(90))
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(LaunchNear, Math.toRadians(-90))
                .waitSeconds(1.5)
                .splineToLinearHeading(PGP_WP, Math.toRadians(90))
                .splineToLinearHeading(PGP, Math.toRadians(90))
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(LaunchNear, Math.toRadians(225))
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(25))
                .splineToLinearHeading(GPP_WP, Math.toRadians(90))
                .splineToLinearHeading(GPP, Math.toRadians(90))
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(LaunchNear, Math.toRadians(180))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(10,20,0),Math.toRadians(0))
                .build();

        return new SequentialAction(driveToLaunch1);
    }
}
