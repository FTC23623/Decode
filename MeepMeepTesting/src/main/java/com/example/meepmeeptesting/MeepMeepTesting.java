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

        // Create a button to run the selected auto
        JButton runButton = new JButton("Run");

        // Create a panel to hold the controls
        JPanel panel = new JPanel();
        panel.add(autoSelector);
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

                // Create a new bot entity for the simulation, setting the color
                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .setColorScheme(isBlue ? new ColorSchemeBlueDark() : new ColorSchemeRedDark())
                        .build();

                // Run the selected auto
                switch (selectedAuto) {
                    case "AutoFarBlue":
                        myBot.runAction(BuildAutoFar(myBot, true));
                        break;
                    case "AutoFarRed":
                        myBot.runAction(BuildAutoFar(myBot, false));
                        break;
                    case "AutoNearBlue":
                        myBot.runAction(BuildAutoNear(myBot, true));
                        break;
                    case "AutoNearRed":
                        myBot.runAction(BuildAutoNear(myBot, false));
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

    private static SequentialAction BuildAutoFar(RoadRunnerBotEntity myBot, boolean flip) {
        Pose2d beginPose = FlipPose(60.0, 15.0, 0.0, flip);

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d Launch = FlipPose(55, 15, -20, flip);
        Pose2d GPP_WP = FlipPose(40, 30, 90, flip);
        Pose2d GPP = FlipPose(40, 54, 90, flip);
        Pose2d PGP_WP = FlipPose(16, 30, 90, flip);
        Pose2d PGP = FlipPose(16, 54, 90, flip);
        Pose2d PPG_WP = FlipPose(-7, 30,90, flip);
        Pose2d PPG = FlipPose(-7, 54, 90, flip);
        Pose2d End = FlipPose(30,10,0, flip);

        Action driveToLaunch1 = myBot.getDrive().actionBuilder(beginPose)
                .setTangent(FlipTangent(180, flip))
                .splineToLinearHeading(Launch, FlipTangent(180, flip))
                .waitSeconds(1.5)
                .setTangent(FlipTangent(180, flip))
                .splineToLinearHeading(GPP_WP, FlipTangent(135, flip))
                .setTangent(FlipTangent(90, flip))
                .splineToLinearHeading(GPP, FlipTangent(90, flip))
                .setTangent(FlipTangent(-90, flip))
                .splineToLinearHeading(Launch, FlipTangent(0, flip))
                .waitSeconds(1.5)
                .setTangent(FlipTangent(180, flip))
                .splineToLinearHeading(PGP_WP, FlipTangent(135, flip))
                .setTangent(FlipTangent(90, flip))
                .splineToLinearHeading(PGP, FlipTangent(90, flip))
                .setTangent(FlipTangent(-90, flip))
                .splineToLinearHeading(Launch, FlipTangent(0, flip))
                .waitSeconds(1.5)
                .setTangent(FlipTangent(180, flip))
                .splineToLinearHeading(PPG_WP, FlipTangent(135, flip))
                .setTangent(FlipTangent(90, flip))
                .splineToLinearHeading(PPG, FlipTangent(90, flip))
                .setTangent(FlipTangent(-90, flip))
                .splineToLinearHeading(Launch, FlipTangent(0, flip))
                .waitSeconds(1.5)
                .splineToLinearHeading(End, FlipTangent(180, flip))
                .build();

        return new SequentialAction(driveToLaunch1);
    }

    private static SequentialAction BuildAutoNear(RoadRunnerBotEntity myBot, boolean flip) {
        Pose2d beginPose = FlipPose(-54, 52, 310, flip);

        // All poses defined for autos on the red side
        // FlipPose and FlipTangent auto adjust for blue
        Pose2d GPP_WP = FlipPose(46, 30, 90, flip);
        Pose2d GPP = FlipPose(46, 54, 90, flip);
        Pose2d PGP_WP = FlipPose(22, 30, 90, flip);
        Pose2d PGP = FlipPose(22, 54, 90, flip);
        Pose2d PPG_WP = FlipPose(-4, 30, 90, flip);
        Pose2d PPG = FlipPose(-4, 54, 90, flip);
        Pose2d LaunchNear = FlipPose(-25, 24, -40, flip);
        Pose2d End = FlipPose(10, 20, 0, flip);

        Action driveToLaunch1 = myBot.getDrive().actionBuilder(beginPose)
                .setTangent(FlipTangent(315, flip))
                .splineToLinearHeading(LaunchNear, FlipTangent(315, flip))
                .waitSeconds(1.5)
                .splineToLinearHeading(PPG_WP, FlipTangent(90, flip))
                .splineToLinearHeading(PPG, FlipTangent(90, flip))
                .setTangent(FlipTangent(225, flip))
                .splineToLinearHeading(LaunchNear, FlipTangent(-90, flip))
                .waitSeconds(1.5)
                .splineToLinearHeading(PGP_WP, FlipTangent(90, flip))
                .splineToLinearHeading(PGP, FlipTangent(90, flip))
                .setTangent(FlipTangent(225, flip))
                .splineToLinearHeading(LaunchNear, FlipTangent(225, flip))
                .waitSeconds(1.5)
                .setTangent(FlipTangent(25, flip))
                .splineToLinearHeading(GPP_WP, FlipTangent(90, flip))
                .splineToLinearHeading(GPP, FlipTangent(90, flip))
                .setTangent(FlipTangent(225, flip))
                .splineToLinearHeading(LaunchNear, FlipTangent(180, flip))
                .waitSeconds(1.5)
                .splineToLinearHeading(End, FlipTangent(0, flip))
                .build();
        return new SequentialAction(driveToLaunch1);
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
