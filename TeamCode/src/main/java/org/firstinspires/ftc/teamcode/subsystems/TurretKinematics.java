package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.objects.VisionResult;
import org.firstinspires.ftc.teamcode.types.Constants;

public class TurretKinematics {
    public static double CalcDistanceToTag(VisionResult vision) {
        double targetOffsetAngle_Vertical = vision.GetYOffset();
        double angleToGoalDegrees = Constants.limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        //calculate distance
        double distanceFromLimelightToGoalInches = (Constants.goalHeightInches - Constants.limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        //mOp.mTelemetry.addData("GoalDistance", distanceFromLimelightToGoalInches);
        return distanceFromLimelightToGoalInches;
    }
}
