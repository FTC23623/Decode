package org.firstinspires.ftc.teamcode.objects;

import org.firstinspires.ftc.teamcode.types.DecodeAprilTag;

public interface VisionResult {
    public abstract double GetXOffset();
    public abstract double GetYOffset();
    public abstract DecodeAprilTag GetTagClass();
    public abstract long GetTimestamp();
}
