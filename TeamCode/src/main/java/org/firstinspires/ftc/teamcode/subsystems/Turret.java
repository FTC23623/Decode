package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
@Config
public class Turret implements Subsystem {
    private final HydraOpMode mOp;
    public static double mPosChangeRate = 0;
    public static double mMaxPos = 0;
    public static double mMinPos = 0;

    public Turret(HydraOpMode opMode) {
        mOp = opMode;
        // TODO: get servo from hardware map
    }

    @Override
    public boolean Init() {
        return true;
    }

    @Override
    public void HandleUserInput() {
        // TODO: capture operator control inputs and store for use in process
    }

    @Override
    public void Process() {
        // TODO: process user input and set the servo position
        // TODO: scale mPosChangeRate with control input value
        // TODO: set position on servo based on scaled input
        // TODO: clamp position to min and max values
    }
}
