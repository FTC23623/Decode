package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "BlueFar2", preselectTeleOp = "TeleBlue")
public class AutoFarBlue2 extends AutoFar {
    public AutoFarBlue2() {
        super(VisionMode.VisionMode_BlueGoal, true, 2);
    }
}
