package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
@Config
public class Turret implements Subsystem {
    private final HydraOpMode mOp;
    public static double mPosChangeRate = 0;
    public static double mMaxPos = 0;
    public static double mMinPos = 0;
    private double UserInput = 0;
    private Servo TurretServo;

    public Turret(HydraOpMode opMode) {
        mOp = opMode;
        TurretServo = mOp.mHardwareMap.get(Servo.class,"TurretServo");
        // TODO: get servo from hardware map
    }

    @Override
    public boolean Init() {
        return true;
    }

    @Override
    public void HandleUserInput() {
        UserInput = mOp.mOperatorGamepad.right_stick_x;
        // TODO: capture operator control inputs and store for use in process
    }

    @Override
    public void Process() {
        // TODO: process user input and set the servo position
        // TODO: scale mPosChangeRate with control input value
        double position_change = UserInput*mPosChangeRate;
        // TODO: set position on servo based on scaled input
        double CurrentPos = TurretServo.getPosition();
        double NewPos = CurrentPos + position_change;
        // TODO: clamp position to min and max values
        NewPos = Math.min(mMaxPos, NewPos);
        NewPos = Math.max(mMinPos, NewPos);
        TurretServo.setPosition(NewPos);
    }
}
