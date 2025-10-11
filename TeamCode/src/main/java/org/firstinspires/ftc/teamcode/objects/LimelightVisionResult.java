package org.firstinspires.ftc.teamcode.objects;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.teamcode.types.AprilTagClass;
import java.util.List;

public class LimelightVisionResult implements VisionResult {
    private final LLResult result;

    public LimelightVisionResult(LLResult result) {
        this.result = result;
    }

    @Override
    public double GetXOffset() {
        return result.getTx();
    }

    @Override
    public double GetYOffset() {
        return result.getTy();
    }

    @Override
    public AprilTagClass GetTagClass() {
        List<LLResultTypes.FiducialResult> fid = result.getFiducialResults();
        if (!fid.isEmpty()) {
            return AprilTagClass.FromInt(fid.get(0).getFiducialId());
        }
        return AprilTagClass.AprilTagClass_Unknown;
    }
}
