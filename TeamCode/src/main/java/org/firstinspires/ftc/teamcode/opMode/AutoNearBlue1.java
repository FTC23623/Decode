package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Disabled
@Autonomous(name = "BlueNear1", preselectTeleOp = "TeleBlue", group = "Blue")
public class AutoNearBlue1 extends AutoNear {
    public AutoNearBlue1() {
        super(VisionMode.VisionMode_BlueGoal, true, 1, 0);
    }
}
