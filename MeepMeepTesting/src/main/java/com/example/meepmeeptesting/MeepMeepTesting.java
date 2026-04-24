package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
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

    private static final double launchTimeS = 1.15;

    public static void main(String[] args) {
        // Create a dropdown menu for selecting the auto
        String[] autos = {"RedNear", "BlueNear", "RedFar", "BlueFar" };
        JComboBox<String> autoSelector = new JComboBox<>(autos);

        // Create a dropdown for selecting the spike count
        Integer[] spikeCounts = { 3, 2, 1, 0 };
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
                    case "RedFar":
                        myBot.runAction(BuildFarAuto(myBot, false, spikeCount));
                        break;
                    case "BlueFar":
                        myBot.runAction(BuildFarAuto(myBot, true, spikeCount));
                        break;
                    case "RedNear":
                        myBot.runAction(BuildNearAuto(myBot, false, spikeCount));
                        break;
                    case "BlueNear":
                        myBot.runAction(BuildNearAuto(myBot, true, spikeCount));
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

    private static SequentialAction LoadingZoneSequence(RoadRunnerBotEntity mDrive, Pose2d LaunchPos, boolean driveToLaunch, boolean flip, Pose2d StartPos, boolean straight, int count) {
        Pose2d LoadingZone;
        Pose2d Slowdown_Pose;
        if (straight) {
            double x = StartPos.position.x;
            if (count > 0) {
                if ((count & 1) != 0) {
                    x -= 6;
                } else {
                    x -= 12;
                }
            }
            LoadingZone = FlipPose(StartPos.position.x,56,90, flip);
            Slowdown_Pose = FlipPose(LoadingZone.position.x, 49, 90, flip);
        }
        else {
            LoadingZone = FlipPose(64,56,90, flip);
            Slowdown_Pose = Waypoint(StartPos, LoadingZone, 0.75);
        }
        // cap velocity when going into the corner of the field
        final double maxVelToCorner = 25;

        // fetch and drive to waypoint
        Action fetch = mDrive.getDrive().actionBuilder(StartPos)
                .setTangent(FlipTangent(90, flip))
                .splineToSplineHeading(Slowdown_Pose, FlipTangent(90, flip))
                .splineToSplineHeading(LoadingZone, FlipTangent(-90, flip), new TranslationalVelConstraint(maxVelToCorner))
                .build();

        // drive to launch position
        Action goToLaunch =  mDrive.getDrive().actionBuilder(StartPos)
                .setTangent(AutoTangent(StartPos.position, Slowdown_Pose.position, flip))
                .splineToSplineHeading(Slowdown_Pose, FlipTangent(90, flip))
                .splineToSplineHeading(LoadingZone, FlipTangent(-90, flip), new TranslationalVelConstraint(maxVelToCorner))
                .splineToLinearHeading(LaunchPos, FlipTangent(-90, flip))
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

    private static SequentialAction BuildFarAuto(RoadRunnerBotEntity myBot, boolean flip, int spikeCount) {
        Pose2d beginPose = FlipPose(64, 28.5, 90, flip);

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        //Pose2d Launch1 = FlipPose(55, 15, 90, flip);
        Vector2d GPPPos = FlipCoordinate(36, 48, flip);
        Vector2d PGPPos = FlipCoordinate(12, 48, flip);
        Vector2d Launch2Pos = FlipCoordinate(59, 21, flip);
        Pose2d GPP = new Pose2d(GPPPos, AutoTangent(Launch2Pos, GPPPos, flip));
        Pose2d PGP = new Pose2d(PGPPos, AutoTangent(Launch2Pos, PGPPos, flip));
        Pose2d Launch2 = new Pose2d(Launch2Pos, FlipTangent(90, flip));
        Pose2d GPPSlowdownPose = Waypoint(Launch2, GPP, 0.75);
        Pose2d PGPSlowdownPose = Waypoint(Launch2, PGP, 0.75);

        Action launchPreloads = myBot.getDrive().actionBuilder(beginPose)
                .waitSeconds(3)
                .build();

        Action fetchGPP = myBot.getDrive().actionBuilder(Launch2)
                .setTangent(GPP.heading)
                .splineToSplineHeading(GPPSlowdownPose, GPP.heading)
                .splineToSplineHeading(GPP, GPP.heading, new TranslationalVelConstraint(25))
                .setTangent(AutoTangent(GPPPos, Launch2Pos, flip))
                .splineToSplineHeading(Launch2, AutoTangent(GPPPos, Launch2Pos, flip))
                .waitSeconds(launchTimeS)
                .build();

        Action fetchPGP = myBot.getDrive().actionBuilder(Launch2)
                .setTangent(PGP.heading)
                .splineToSplineHeading(PGPSlowdownPose, PGP.heading)
                .splineToSplineHeading(PGP, PGP.heading, new TranslationalVelConstraint(25))
                .setTangent(AutoTangent(PGPPos, Launch2Pos, flip))
                .splineToSplineHeading(Launch2, AutoTangent(PGPPos, Launch2Pos, flip))
                .waitSeconds(launchTimeS)
                .build();

        int lzcount = 0;
        SequentialAction ret =  new SequentialAction(
                launchPreloads,
                LoadingZoneSequence(myBot, Launch2, true, flip, beginPose, false, lzcount++)
        );
        if (spikeCount > 0) {
            ret = new SequentialAction(
                    ret,
                    fetchGPP
            );
        }
        if (spikeCount > 1) {
            ret = new SequentialAction(
                    ret,
                    fetchPGP
            );
        }
        int lzPickups = 5 - spikeCount;
        if (spikeCount > 0) {
            //--lzPickups;
        }
        for (int i = 0; i < lzPickups; ++i) {
            ret = new SequentialAction(
                    ret,
                    LoadingZoneSequence(myBot, Launch2, true, flip, Launch2, true, lzcount++)
            );
        }

        ret = new SequentialAction(
                ret,
                LoadingZoneSequence(myBot, Launch2, false, flip, Launch2, true, 0)
        );

        return ret;
    }

    private static SequentialAction BuildNearAuto(RoadRunnerBotEntity myBot, boolean flip, int spikeCount) {
        Pose2d beginPose = FlipPose(-62.5, 39, 0, flip);

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d Launch1 = FlipPose(-12, 20, 45, flip);
        Vector2d PPGPos = FlipCoordinate(-12, 48, flip);
        Vector2d PGPPos = FlipCoordinate(12, 50, flip);
        Vector2d GPPPos = FlipCoordinate(36, 48, flip);
        Pose2d Gate = FlipPose(2, 54, 90, flip);
        Pose2d PPG = new Pose2d(PPGPos, AutoTangent(Launch1.position, PPGPos, flip));
        Pose2d PGP = new Pose2d(PGPPos, AutoTangent(Launch1.position, PGPPos, flip));
        Pose2d GPP = new Pose2d(GPPPos, AutoTangent(Launch1.position, GPPPos, flip));
        Pose2d PPGSlowdownPose = Waypoint(Launch1, PPG, 0.75);
        Pose2d PGPSlowdownPose = Waypoint(Launch1, PGP, 0.75);
        Pose2d GPPSlowdownPose = Waypoint(Launch1, GPP, 0.75);
        Pose2d GateFeed = FlipPose(12, 56, 135, flip);

        double slowdownspeed = 20;
        double preloadtangent = AutoTangent(beginPose.position, Launch1.position, flip);
        double fromgatetangent = AutoTangent(Gate.position, Launch1.position, flip);
        double togatetangent = AutoTangent(Launch1.position, Gate.position, flip);
        double fromgatefeedtangent = AutoTangent(GateFeed.position, Launch1.position, flip);

        Action launchPreload = myBot.getDrive().actionBuilder(beginPose)
                .setTangent(preloadtangent)
                .splineToLinearHeading(Launch1, preloadtangent)
                .waitSeconds(2)
                .build();

        Action fetchPPG = myBot.getDrive().actionBuilder(Launch1)
                .setTangent(PPG.heading)
                .splineToSplineHeading(PPGSlowdownPose, PPG.heading)
                .splineToSplineHeading(PPG, PPG.heading, new TranslationalVelConstraint(slowdownspeed))
                .setTangent(FlipTangent(0, flip))
                .splineToLinearHeading(Gate, FlipTangent(90, flip))
                .waitSeconds(0.5)
                .setTangent(fromgatetangent)
                .splineToLinearHeading(Launch1, fromgatetangent)
                .waitSeconds(launchTimeS)
                .build();

        Action fetchPGP = myBot.getDrive().actionBuilder(Launch1)
                .setTangent(PGP.heading)
                .splineToSplineHeading(PGPSlowdownPose, PGP.heading)
                .splineToSplineHeading(PGP, PGP.heading, new TranslationalVelConstraint(slowdownspeed))
                .setTangent(FlipTangent(180, flip))
                .splineToLinearHeading(Gate, FlipTangent(90, flip))
                .waitSeconds(0.5)
                .setTangent(fromgatetangent)
                .splineToSplineHeading(Launch1, fromgatetangent)
                .waitSeconds(launchTimeS)
                .build();

        Action fetchGPP = myBot.getDrive().actionBuilder(Launch1)
                .setTangent(GPP.heading)
                .splineToSplineHeading(GPPSlowdownPose, GPP.heading)
                .splineToSplineHeading(GPP, GPP.heading, new TranslationalVelConstraint(slowdownspeed))
                .setTangent(AutoTangent(GPPPos, Launch1.position, flip))
                .splineToSplineHeading(Launch1, AutoTangent(GPPPos, Launch1.position, flip))
                .waitSeconds(launchTimeS)
                .build();

        Action gateFeed = myBot.getDrive().actionBuilder(Launch1)
                .setTangent(FlipTangent(0, flip))
                .splineToLinearHeading(Gate, FlipTangent(90, flip))
                .setTangent(FlipTangent(-90, flip))
                .splineToLinearHeading(GateFeed, FlipTangent(45, flip))
                .waitSeconds(0.5)
                .setTangent(fromgatefeedtangent)
                .splineToLinearHeading(Launch1, fromgatefeedtangent)
                .waitSeconds(launchTimeS)
                .build();

        Action park = myBot.getDrive().actionBuilder(Launch1)
                .setTangent(PPG.heading)
                .splineToLinearHeading(PPG, PPG.heading)
                .build();

        // This logic mirrors the construction in your AutoFar.java
        SequentialAction ret =  new SequentialAction(
                launchPreload
        );
        if (spikeCount > 2) {
            ret = new SequentialAction(
                ret,
                fetchPPG,
                fetchPGP,
                fetchGPP
            );
        } else {
            ret = new SequentialAction(
                ret,
                fetchPGP,
                gateFeed,
                fetchPPG
            );
        }
        ret = new SequentialAction(
                ret,
                park
        );
        return ret;
    }

    private static Pose2d FlipPose(double x, double y, double heading, boolean flip) {
        if (flip) {
            return new Pose2d(x, -y, Math.toRadians(-heading));
        }
        return new Pose2d(x, y, Math.toRadians(heading));
    }

    private static Vector2d FlipCoordinate(double x, double y, boolean flip) {
        if (flip) {
            return new Vector2d(x, -y);
        }
        return new Vector2d(x, y);
    }

    private static double FlipTangent(double tangent, boolean flip) {
        if (flip) {
            return Math.toRadians(-tangent);
        }
        return Math.toRadians(tangent);
    }

    private static double AutoTangent(Vector2d start, Vector2d end, boolean flip) {
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double ret;
        ret = Math.atan2(dy, dx);
        if (flip) {
            return ret;
        } else {
            return ret;
        }
    }

    private static Pose2d Waypoint(Pose2d start, Pose2d end, double fraction) {
        double slowdown_x = start.position.x + fraction * (end.position.x - start.position.x);
        double slowdown_y = start.position.y + fraction * (end.position.y - start.position.y);
        return new Pose2d(slowdown_x, slowdown_y, end.heading.toDouble());
    }
}
