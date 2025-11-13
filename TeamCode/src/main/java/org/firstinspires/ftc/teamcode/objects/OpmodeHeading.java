package org.firstinspires.ftc.teamcode.objects;

import com.acmerobotics.roadrunner.Pose2d;

public class OpmodeHeading {
    static private Pose2d mYawOffset = null;

    public static void SetOffset(Pose2d offset) {
        mYawOffset = offset;
    }

    public static Pose2d GetOffset() {
        return mYawOffset;
    }

    public static boolean handOff = false;
}
