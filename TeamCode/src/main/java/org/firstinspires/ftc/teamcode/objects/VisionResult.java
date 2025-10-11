package org.firstinspires.ftc.teamcode.objects;

import org.firstinspires.ftc.teamcode.types.AprilTagClass;

public interface VisionResult {
    public abstract double GetXOffset();
    public abstract double GetYOffset();
    public abstract AprilTagClass GetTagClass();
}
