package org.firstinspires.ftc.teamcode.objects;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorSlewRate {
    static public void Set(DcMotor motor, double power, double slewRate) {
        double currentPower = motor.getPower();
        double desiredChange = power - currentPower;
        double limitedChange = Math.max(-slewRate, Math.min(desiredChange, slewRate));
        motor.setPower(currentPower + limitedChange);
    }
}
