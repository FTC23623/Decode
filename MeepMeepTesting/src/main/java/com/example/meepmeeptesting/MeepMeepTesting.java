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

        Pose2d Launch=new Pose2d(55, 15, Math.toRadians(-20));
        Pose2d GPP_WP=new Pose2d(35, 30,Math.toRadians(90));
        Pose2d GPP=new Pose2d(35,48,Math.toRadians(90));
        Pose2d PGP_WP=new Pose2d(11, 30,Math.toRadians(90));
        Pose2d PGP=new Pose2d(11,48,Math.toRadians(90));
        Pose2d PPG_WP=new Pose2d(-12, 30,Math.toRadians(90));
        Pose2d PPG=new Pose2d(-12,48,Math.toRadians(90));

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(63, 15, Math.toRadians(0)))
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
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}