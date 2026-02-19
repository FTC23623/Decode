package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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

    private static final double launchTimeS = 1.3;

    public static void main(String[] args) {
        // Create a dropdown menu for selecting the auto
        String[] autos = {"FarFetch", "AutoFarBlue", "AutoFarRed", "AutoNearBlue", "AutoNearRed", "AutoNearBlueGate", "AutoNearRedGate"};
        JComboBox<String> autoSelector = new JComboBox<>(autos);

        // Create a dropdown for selecting the spike count
        Integer[] spikeCounts = { 4, 3, 2, 1};
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
                        .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                        .setColorScheme(isBlue ? new ColorSchemeBlueDark() : new ColorSchemeRedDark())
                        .setDimensions(16.2, 17.7)
                        .build();

                // Run the selected auto
                switch (selectedAuto) {
                    case "AutoFarBlue":
                        myBot.runAction(BuildAutoFar(myBot, true, spikeCount, false));
                        break;
                    case "AutoFarRed":
                        myBot.runAction(BuildAutoFar(myBot, false, spikeCount, false));
                        break;
                    case "AutoNearBlue":
                        myBot.runAction(BuildAutoNear(myBot, true, spikeCount, false));
                        break;
                    case "AutoNearRed":
                        myBot.runAction(BuildAutoNear(myBot, false, spikeCount, false));
                        break;
                    case "AutoNearBlueGate":
                        myBot.runAction(BuildAutoNear(myBot, true, spikeCount, true));
                        break;
                    case "AutoNearRedGate":
                        myBot.runAction(BuildAutoNear(myBot, false, spikeCount, true));
                        break;
                    case "FarFetch":
                        myBot.runAction(BuildFarFetch(myBot,false, spikeCount,false));
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

    private static SequentialAction BuildAutoFar(RoadRunnerBotEntity myBot, boolean flip, int spikeCount, boolean loop) {
        Pose2d beginPose = FlipPose(60.0, 15.0, 0.0, flip);

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d Launch = FlipPose(55, 15, -20, flip);
        Pose2d GPP_WP = FlipPose(34, 30, 90, flip);
        Pose2d GPPStraight = FlipPose(46, 56, 90, flip);
        Pose2d GPPLoop = FlipPose(46, 56, 90, flip);
        Pose2d PGP_WP = FlipPose(12, 30, 90, flip);
        Pose2d PGPStraight = FlipPose(24, 56, 90, flip);
        Pose2d PGPLoop = FlipPose(24, 56, 90, flip);
        Pose2d PPG_WP = FlipPose(-12, 35, 90, flip);
        Pose2d PPG = FlipPose(-12, 48, 90, flip);
        Pose2d End = FlipPose(30, 15, 0, flip);

        Pose2d GPP;
        Pose2d PGP;
        double spikeTangent;
        if (loop) {
            GPP = GPPLoop;
            PGP = PGPLoop;
            spikeTangent = 0;
        } else {
            GPP = GPPStraight;
            PGP = PGPStraight;
            spikeTangent = 90;
        }

        Action launchPreload = myBot.getDrive().actionBuilder(beginPose)
                .splineToSplineHeading(Launch, FlipTangent(180, flip))
                .waitSeconds(2)
                .build();

        Action fetchGPP = myBot.getDrive().actionBuilder(Launch)
                .setTangent(FlipTangent(180, flip))
                .splineToSplineHeading(GPP_WP, FlipTangent(90, flip))
                .setTangent(FlipTangent(90, flip))
                .splineToSplineHeading(GPP, FlipTangent(spikeTangent, flip))
                .setTangent(FlipTangent(-spikeTangent, flip))
                .splineToLinearHeading(Launch, FlipTangent(-90, flip))
                .waitSeconds(launchTimeS)
                .build();

        Action fetchPGP = myBot.getDrive().actionBuilder(Launch)
                .setTangent(FlipTangent(180, flip))
                .splineToSplineHeading(PGP_WP, FlipTangent(90, flip))
                .setTangent(FlipTangent(90, flip))
                .splineToSplineHeading(PGP, FlipTangent(spikeTangent, flip))
                .setTangent(FlipTangent(-spikeTangent, flip))
                .splineToLinearHeading(Launch, FlipTangent(-60, flip))
                .waitSeconds(launchTimeS)
                .build();

        // This logic mirrors the construction in your AutoFar.java
        SequentialAction ret =  new SequentialAction(
                launchPreload,
                LoadingZoneSequence(myBot, Launch, true, flip),
                fetchGPP
        );
        if (spikeCount > 1) {
            ret = new SequentialAction(
                    ret,
                    fetchPGP
            );
        } else {
            ret = new SequentialAction(
                    ret,
                    LoadingZoneSequence(myBot, Launch, true, flip),
                    LoadingZoneSequence(myBot, Launch, true, flip)
            );
        }
        ret = new SequentialAction(ret, LoadingZoneSequence(myBot, Launch, false, flip));
        return ret;
    }

    private static SequentialAction BuildAutoNear(RoadRunnerBotEntity myBot, boolean flip, int spikeCount, boolean gate) {
        Pose2d beginPose = FlipPose(-54, 52, 310, flip);

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d GPP_WP = FlipPose(34, 35, 90, flip);
        Pose2d GPP = FlipPose(34, 56, 90, flip);
        Pose2d PGP_WP = FlipPose(12, 35, 90, flip);
        Pose2d PGP = FlipPose(12, 56, 90, flip);
        Pose2d PPG_WP = FlipPose(-12, 35, 90, flip);
        Pose2d PPG = FlipPose(-12, 52, 90, flip);
        Pose2d LaunchNear = FlipPose(-25, 24, -40, flip);
        Pose2d End = FlipPose(-14, 36, -40, flip);
        Pose2d GateWP = FlipPose(-6, 50, 180, flip);
        Pose2d Gate = FlipPose(-6, 60, 180, flip);
        Pose2d GateWP2 = FlipPose(4, 50, 0, flip);
        Pose2d Gate2 = FlipPose(4, 59, 0, flip);

        Action driveToLaunchPreload = myBot.getDrive().actionBuilder(beginPose)
                .setTangent(FlipTangent(315, flip))
                .splineToSplineHeading(LaunchNear, FlipTangent(315, flip))
                .waitSeconds(1.5)
                .build();

        Action fetchPPG;
        if (gate) {
            fetchPPG = myBot.getDrive().actionBuilder(LaunchNear)
                    .setTangent(FlipTangent(0, flip))
                    .splineToSplineHeading(PPG_WP, FlipTangent(90, flip))
                    .splineToSplineHeading(PPG, FlipTangent(90, flip))
                    .setTangent(FlipTangent(0, flip))
                    .splineToSplineHeading(GateWP, FlipTangent(-90, flip))
                    .splineToSplineHeading(Gate, FlipTangent(90, flip))
                    .waitSeconds(1.5)
                    .setTangent(FlipTangent(-90,flip))
                    .splineToLinearHeading(LaunchNear, FlipTangent(-180, flip))
                    .waitSeconds(launchTimeS)
                    .build();
        } else {
            fetchPPG = myBot.getDrive().actionBuilder(LaunchNear)
                    .setTangent(FlipTangent(0, flip))
                    .splineToSplineHeading(PPG_WP, FlipTangent(90, flip))
                    .splineToSplineHeading(PPG, FlipTangent(90, flip))
                    .setTangent(FlipTangent(-90, flip))
                    .splineToLinearHeading(LaunchNear, FlipTangent(-180, flip))
                    .waitSeconds(launchTimeS)
                    .build();
        }

        Action fetchPGP;
        if (gate) {
            fetchPGP = myBot.getDrive().actionBuilder(LaunchNear)
                    .setTangent(FlipTangent(0, flip))
                    .splineToSplineHeading(PGP_WP, FlipTangent(90, flip))
                    .splineToSplineHeading(PGP, FlipTangent(90, flip))
                    .setTangent(FlipTangent(-150, flip))
                    .splineToSplineHeading(GateWP2, FlipTangent(-90, flip))
                    .splineToSplineHeading(Gate2, FlipTangent(90, flip))
                    .waitSeconds(1.5)
                    .setTangent(FlipTangent(-90, flip))
                    .splineToLinearHeading(LaunchNear, FlipTangent(-135, flip))
                    .waitSeconds(launchTimeS)
                    .build();
        } else {
            fetchPGP = myBot.getDrive().actionBuilder(LaunchNear)
                    .setTangent(FlipTangent(0, flip))
                    .splineToSplineHeading(PGP_WP, FlipTangent(90, flip))
                    .splineToSplineHeading(PGP, FlipTangent(90, flip))
                    .setTangent(FlipTangent(-90, flip))
                    .splineToLinearHeading(LaunchNear, FlipTangent(-135, flip))
                    .waitSeconds(launchTimeS)
                    .build();
        }

        Action pickupGPP = myBot.getDrive().actionBuilder(LaunchNear)
                .setTangent(FlipTangent(0, flip))
                .splineToSplineHeading(GPP_WP, FlipTangent(90, flip))
                .splineToSplineHeading(GPP, FlipTangent(90, flip))
                .setTangent(FlipTangent(-90, flip))
                .splineToSplineHeading(LaunchNear, FlipTangent(-135, flip))
                .waitSeconds(launchTimeS)
                .build();

        Action driveToEnd = myBot.getDrive().actionBuilder(LaunchNear)
                .setTangent(FlipTangent(40, flip))
                .splineToLinearHeading(End, FlipTangent(40, flip))
                .build();

        // This logic mirrors the construction in your AutoNear.java
        SequentialAction ret = new SequentialAction(driveToLaunchPreload, fetchPPG);
        if (spikeCount > 1) {
            ret = new SequentialAction(ret, fetchPGP);
        }
        if (spikeCount > 2) {
            ret = new SequentialAction(ret, pickupGPP);
        }
        ret = new SequentialAction(ret, driveToEnd);
        return ret;
    }
    private static SequentialAction BuildFarFetch(RoadRunnerBotEntity myBot, boolean flip, int spikeCount, boolean gate) {
        Pose2d beginPose = FlipPose(60.0, 15.0, 0.0, flip);

        Pose2d Launch = FlipPose(55, 15, -20, flip);

        Action launchPreload = myBot.getDrive().actionBuilder(beginPose)
                .setTangent(FlipTangent(180, flip))
                .splineToSplineHeading(Launch, FlipTangent(180, flip))
                .waitSeconds(2)
                .build();

        SequentialAction ret =  new SequentialAction(
                launchPreload,
                LoadingZoneSequence(myBot, Launch, true, flip)
        );
        int count = spikeCount;
        while (count > 0) {
            count--;
            double waitTime = 0;
            if (count == 1) {
                waitTime = 0.25;
            }
            ret = new SequentialAction(
                    ret,
                    myBot.getDrive().actionBuilder(Launch).waitSeconds(waitTime).build(),
                    LoadingZoneSequence(myBot, Launch, count > 0, flip)
            );
        }
        return ret;
    }

    private static SequentialAction LoadingZoneSequence(RoadRunnerBotEntity mDrive, Pose2d LaunchPos, boolean driveToLaunch, boolean flip) {
        Pose2d LoadingZone = FlipPose(59,53,90, flip);
        Pose2d LoadingZone_WP= FlipPose(59, 40, 90, flip);
        Pose2d LoadingZone_WP2= FlipPose(59, 50, 90, flip);

        // cap velocity when going into the corner of the field
        final double maxVelToCorner = 25;

        // fetch and drive to waypoint
        Action fetch = mDrive.getDrive().actionBuilder(LaunchPos)
                .setTangent(FlipTangent(90, flip))
                .splineToSplineHeading(LoadingZone_WP, FlipTangent(90, flip))
                .splineToSplineHeading(LoadingZone, FlipTangent(-90, flip), new TranslationalVelConstraint(maxVelToCorner))
                .splineToSplineHeading(LoadingZone_WP, FlipTangent(-90, flip))
                .build();

        // drive to launch position
        Action goToLaunch =  mDrive.getDrive().actionBuilder(LaunchPos)
                .setTangent(FlipTangent(90, flip))
                .splineToSplineHeading(LoadingZone_WP, FlipTangent(90, flip))
                .splineToSplineHeading(LoadingZone, FlipTangent(-90, flip), new TranslationalVelConstraint(maxVelToCorner))
                .splineToSplineHeading(LoadingZone_WP2, FlipTangent(-90, flip))
                .splineToSplineHeading(LaunchPos, FlipTangent(-90, flip))
                .waitSeconds(launchTimeS)
                .build();

        // if we're launching, continue to launch point, re-enable turret and launch
        // otherwise stop and re-enable turret
        if (driveToLaunch) {
            return new SequentialAction(
                    goToLaunch
            );
        } else {
            return new SequentialAction(
                    fetch
            );
        }
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
