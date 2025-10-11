package org.firstinspires.ftc.teamcode.objects;

import org.firstinspires.ftc.teamcode.types.VisionMode;

public interface Vision extends Subsystem {
    public abstract void Init();
    public abstract void Process();
    public abstract VisionResult GetResult();
    public abstract void SetMode(VisionMode mode);
}
