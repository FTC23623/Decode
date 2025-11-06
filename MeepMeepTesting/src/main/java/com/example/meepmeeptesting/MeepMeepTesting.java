package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class MeepMeepTesting {

    public static void main(String[] args) {
        // Create a dropdown menu for selecting the auto
        String[] autos = {"AutoFarBlue", "AutoFarRed", "AutoNearBlue", "AutoNearRed"};
        JComboBox<String> autoSelector = new JComboBox<>(autos);

        // Create a dropdown for selecting the spike count
        Integer[] spikeCounts = {1, 2, 3};
        JComboBox<Integer> spikeSelector = new JComboBox<>(spikeCounts);

        // Create a button to run the selected auto
        JButton runButton = new JButton("Run");

        // Create a panel to hold the controls
        JPanel panel = new JPanel();
        panel.add(new JLabel("Auto:"));
        panel.add(autoSelector);
        panel.add(new JLabel("Spikes:"));
        panel.add(spikeSelector);
        panel.add(runButton);

        // Create a separate frame for the controls
        JFrame frame = new JFrame("MeepMeep Controls");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.getContentPane().add(panel, BorderLayout.CENTER);
        frame.pack();
        frame.setVisible(true);

        runButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                // Create a new MeepMeep instance for each simulation.
                // This will open a new window for each run.
                MeepMeep meepMeep = new MeepMeep(800);

                // Get the selected auto from the dropdown
                String selectedAuto = (String) autoSelector.getSelectedItem();
                boolean isBlue = selectedAuto.contains("Blue");
                int spikeCount = (Integer) spikeSelector.getSelectedItem();

                // Create a new bot entity for the simulation, setting the color
                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .setColorScheme(isBlue ? new ColorSchemeBlueDark() : new ColorSchemeRedDark())
                        .build();

                // Run the selected auto
                switch (selectedAuto) {
                    case "AutoFarBlue":
                        myBot.runAction(BuildAutoFar(myBot, true, spikeCount));
                        break;
                    case "AutoFarRed":
                        myBot.runAction(BuildAutoFar(myBot, false, spikeCount));
                        break;
                    case "AutoNearBlue":
                        myBot.runAction(BuildAutoNear(myBot, true, spikeCount));
                        break;
                    case "AutoNearRed":
                        myBot.runAction(BuildAutoNear(myBot, false, spikeCount));
                        break;
                }

                // Add the new bot to the simulation and start it
                meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                        .setDarkMode(true)
                        .setBackgroundAlpha(0.95f)
                        .addEntity(myBot)
                        .start();
            }
        });
    }

    private static SequentialAction BuildAutoFar(RoadRunnerBotEntity myBot, boolean flip, int spikeCount) {
        Pose2d beginPose = FlipPose(60.0, 15.0, 0.0, flip);

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d Launch = FlipPose(55, 15, -20, flip);
        Pose2d GPP_WP = FlipPose(34, 30, 90, flip);
        Pose2d GPP = FlipPose(34, 56, 90, flip);
        Pose2d PGP_WP = FlipPose(12, 30, 90, flip);
        Pose2d PGP = FlipPose(12, 56, 90, flip);
        Pose2d PPG_WP = FlipPose(-12, 35, 90, flip);
        Pose2d PPG = FlipPose(-12, 48, 90, flip);
        Pose2d End = FlipPose(30, 15, 0, flip);

        Action launchPreload = myBot.getDrive().actionBuilder(beginPose)
                .splineToSplineHeading(Launch, FlipTangent(180, flip))
                .waitSeconds(2)
                .build();

        Action fetchGPP = myBot.getDrive().actionBuilder(Launch)
                .waitSeconds(2)
                .setTangent(FlipTangent(180, flip))
                .splineToSplineHeading(GPP_WP, FlipTangent(90, flip))
                .setTangent(FlipTangent(90, flip))
                .splineToSplineHeading(GPP, FlipTangent(90, flip))
                .setTangent(FlipTangent(-90, flip))
                .splineToSplineHeading(Launch, FlipTangent(-90, flip))
                .waitSeconds(1.5)
                .build();

        Action fetchPGP = myBot.getDrive().actionBuilder(Launch)
                .setTangent(FlipTangent(180, flip))
                .splineToSplineHeading(PGP_WP, FlipTangent(90, flip))
                .setTangent(FlipTangent(90, flip))
                .splineToSplineHeading(PGP, FlipTangent(90, flip))
                .setTangent(FlipTangent(-90, flip))
                .splineToSplineHeading(Launch, FlipTangent(-60, flip))
                .waitSeconds(1.5)
                .build();

        Action pickupPPG = myBot.getDrive().actionBuilder(Launch)
                .setTangent(FlipTangent(180, flip))
                .splineToSplineHeading(PPG_WP, FlipTangent(90, flip))
                .setTangent(FlipTangent(90, flip))
                .splineToSplineHeading(PPG, FlipTangent(90, flip))
                .build();

        Action driveToEnd = myBot.getDrive().actionBuilder(Launch)
                .setTangent(FlipTangent(180, flip))
                .splineToSplineHeading(End, FlipTangent(180, flip))
                .build();

        // This logic mirrors the construction in your AutoFar.java
        SequentialAction ret =  new SequentialAction(launchPreload, fetchGPP);
        if (spikeCount > 1) {
            ret = new SequentialAction(ret, fetchPGP);
        }
        if (spikeCount > 2) {
            ret = new SequentialAction(ret, pickupPPG);
        }
        if (spikeCount < 3) {
            ret = new SequentialAction(ret, driveToEnd);
        }
        return ret;
    }

    private static SequentialAction BuildAutoNear(RoadRunnerBotEntity myBot, boolean flip, int spikeCount) {
        Pose2d beginPose = FlipPose(-54, 52, 310, flip);

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d GPP_WP = FlipPose(34, 30, 90, flip);
        Pose2d GPP = FlipPose(34, 56, 90, flip);
        Pose2d PGP_WP = FlipPose(12, 30, 90, flip);
        Pose2d PGP = FlipPose(12, 56, 90, flip);
        Pose2d PPG_WP = FlipPose(-12, 35, 90, flip);
        Pose2d PPG = FlipPose(-12, 52, 90, flip);
        Pose2d LaunchNear = FlipPose(-25, 24, -40, flip);
        Pose2d End = FlipPose(-25, 52, -90, flip);

        Action driveToLaunchPreload = myBot.getDrive().actionBuilder(beginPose)
                .setTangent(FlipTangent(315, flip))
                .splineToSplineHeading(LaunchNear, FlipTangent(315, flip))
                .waitSeconds(1.5)
                .build();

        Action fetchPPG = myBot.getDrive().actionBuilder(LaunchNear)
                .setTangent(FlipTangent(0, flip))
                .splineToSplineHeading(PPG_WP, FlipTangent(90, flip))
                .splineToSplineHeading(PPG, FlipTangent(90, flip))
                .setTangent(FlipTangent(-90, flip))
                .splineToSplineHeading(LaunchNear, FlipTangent(-180, flip))
                .waitSeconds(1.5)
                .build();

        Action fetchPGP = myBot.getDrive().actionBuilder(LaunchNear)
                .setTangent(FlipTangent(0, flip))
                .splineToSplineHeading(PGP_WP, FlipTangent(90, flip))
                .splineToSplineHeading(PGP, FlipTangent(90, flip))
                .setTangent(FlipTangent(-90, flip))
                .splineToSplineHeading(LaunchNear, FlipTangent(-135, flip))
                .waitSeconds(1.5)
                .build();

        Action pickupGPP = myBot.getDrive().actionBuilder(LaunchNear)
                .setTangent(FlipTangent(0, flip))
                .splineToSplineHeading(GPP_WP, FlipTangent(90, flip))
                .splineToSplineHeading(GPP, FlipTangent(90, flip))
                //.splineToSplineHeading(LaunchNear, FlipTangent(-135, flip))
                //.waitSeconds(1.5)
                //.splineToSplineHeading(End, FlipTangent(90, flip))
                .build();

        Action driveToEnd = myBot.getDrive().actionBuilder(LaunchNear)
                .setTangent(FlipTangent(90, flip))
                .splineToSplineHeading(End, FlipTangent(90, flip))
                .build();

        // This logic mirrors the construction in your AutoNear.java
        SequentialAction ret = new SequentialAction(driveToLaunchPreload, fetchPPG);
        if (spikeCount > 1) {
            ret = new SequentialAction(ret, fetchPGP);
        }
        if (spikeCount > 2) {
            ret = new SequentialAction(ret, pickupGPP);
        }
        if (spikeCount < 3) {
            ret = new SequentialAction(ret, driveToEnd);
        }
        return ret;
    }

    private static Pose2d FlipPose(double x, double y, double heading, boolean flip) {
        if (flip) {
            return new Pose2d(x, -y, Math.toRadians(-heading));
        }
        return new Pose2d(x, y, Math.toRadians(heading));
    }

    private static double FlipTangent(double tangent, boolean flip) {
        if (flip) {
            return Math.toRadians(-tangent);
        }
        return Math.toRadians(tangent);
    }
}
