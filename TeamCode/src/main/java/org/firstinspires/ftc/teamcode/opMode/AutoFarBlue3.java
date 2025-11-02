package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "BlueFar3", preselectTeleOp = "TeleBlue")
public class AutoFarBlue3 extends AutoFar {
    public AutoFarBlue3() {
        super(VisionMode.VisionMode_BlueGoal, true, 3);
    }
}
