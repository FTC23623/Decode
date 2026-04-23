package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "BlueFar0", preselectTeleOp = "TeleBlue", group = "Blue")
public class AutoFarBlue0 extends AutoFar {
    public AutoFarBlue0() {
        super(VisionMode.VisionMode_BlueGoal, true, 0);
    }
}
