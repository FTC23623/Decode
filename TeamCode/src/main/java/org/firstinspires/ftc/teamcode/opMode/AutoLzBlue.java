package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.types.VisionMode;

@Autonomous(name = "BlueLz", preselectTeleOp = "TeleBlue", group = "Blue")
public class AutoLzBlue extends AutoLoadingZone {
    public AutoLzBlue() {
        super(VisionMode.VisionMode_BlueGoal, true, 4);
    }
}
