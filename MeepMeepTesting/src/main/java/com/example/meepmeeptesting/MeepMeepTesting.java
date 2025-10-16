package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d GPP_WP=new Pose2d(46, 30,Math.toRadians(90));
        Pose2d GPP=new Pose2d(46,54,Math.toRadians(90));
        Pose2d PGP_WP=new Pose2d(22, 30,Math.toRadians(90));
        Pose2d PGP=new Pose2d(22,54,Math.toRadians(90));
        Pose2d PPG_WP=new Pose2d(-4, 30,Math.toRadians(90));
        Pose2d PPG=new Pose2d(-4,54,Math.toRadians(90));

        if (false) {
            Pose2d LaunchFar=new Pose2d(55, 15, Math.toRadians(-20));
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 15, Math.toRadians(0)))
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(LaunchFar, Math.toRadians(180))
                    .waitSeconds(1.5)
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(GPP_WP, Math.toRadians(135))
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(GPP, Math.toRadians(90))
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(LaunchFar, Math.toRadians(0))
                    .waitSeconds(1.5)
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(PGP_WP, Math.toRadians(135))
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(PGP, Math.toRadians(90))
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(LaunchFar, Math.toRadians(0))
                    .waitSeconds(1.5)
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(PPG_WP, Math.toRadians(135))
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(PPG, Math.toRadians(90))
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(LaunchFar, Math.toRadians(0))
                    .waitSeconds(1.5)
                    .splineToLinearHeading(new Pose2d(30,10,0),Math.toRadians(180))
                    .build());
        } else {
            Pose2d LaunchNear=new Pose2d(-25, 24, Math.toRadians(-40));
            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-54, 52, Math.toRadians(310)))
                    .setTangent(Math.toRadians(315))
                    .splineToLinearHeading(LaunchNear, Math.toRadians(315))
                    .waitSeconds(1.5)
                    .splineToLinearHeading(PPG_WP, Math.toRadians(90))
                    .splineToLinearHeading(PPG, Math.toRadians(90))
                    .setTangent(Math.toRadians(225))
                    .splineToLinearHeading(LaunchNear, Math.toRadians(-90))
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
                    .build());
        }
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}