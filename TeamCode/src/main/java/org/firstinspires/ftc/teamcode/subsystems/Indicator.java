package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;
import org.firstinspires.ftc.teamcode.types.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.types.VisionMode;

import java.util.ArrayList;

public class Indicator implements Subsystem {
    private final Servo led;
    private final Launcher launcher;
    private final Turret turret;
    private final ElapsedTime timer;
    private final int onTimeMs = 100;
    private final int offTimeMs = 50;
    private final int onTimeIdle = 250;
    private boolean on;
    private int idleState;

    public Indicator(HydraOpMode opMode, Launcher launcher, Turret turret) {
        this.launcher = launcher;
        this.turret = turret;
        this.led = opMode.mHardwareMap.get(Servo.class, "led");
        SetColor(IndicatorColor.INDICATOR_OFF);
        on = false;
        idleState = 0;
        timer = new ElapsedTime();
        timer.reset();
    }

    @Override
    public boolean Init() {
        return true;
    }

    @Override
    public void Process() {
        boolean targetLocked = turret.Locked();
        boolean launcherSpeed = launcher.AtSpeed();
        if (on && timer.milliseconds() > onTimeMs) {
            SetColor(IndicatorColor.INDICATOR_OFF);
            on = false;
            timer.reset();
        } else if (timer.milliseconds() > offTimeMs) {
            if (targetLocked && launcherSpeed) {
                SetColor(IndicatorColor.INDICATOR_GREEN);
            } else if (targetLocked) {
                SetColor(IndicatorColor.INDICATOR_YELLOW);
            } else if (launcherSpeed) {
                SetColor(IndicatorColor.INDICATOR_BLUE);
            } else {
                SetColor(IndicatorColor.INDICATOR_RED);
            }
            on = true;
            timer.reset();
        }
    }

    @Override
    public void HandleUserInput() {

    }

    public void WaitForStart(VisionMode alliance, DecodeAprilTag motif) {
        switch (idleState) {
            case 0:
                if (timer.milliseconds() > offTimeMs) {
                    if (alliance == VisionMode.VisionMode_BlueGoal) {
                        SetColor(IndicatorColor.INDICATOR_BLUE);
                    } else if (alliance == VisionMode.VisionMode_RedGoal){
                        SetColor(IndicatorColor.INDICATOR_RED);
                    } else {
                        SetColor(IndicatorColor.INDICATOR_WHITE);
                    }
                    timer.reset();
                    ++idleState;
                }
                break;
            case 2:
                if (timer.milliseconds() > offTimeMs) {
                    switch (motif) {
                        case DecodeTag_Obelisk_GPP:
                            SetColor(IndicatorColor.INDICATOR_GREEN);
                            break;
                        case DecodeTag_Obelisk_PGP:
                        case DecodeTag_Obelisk_PPG:
                            SetColor(IndicatorColor.INDICATOR_INDIGO);
                            break;
                        default:
                            SetColor(IndicatorColor.INDICATOR_WHITE);
                            break;
                    }
                    timer.reset();
                    ++idleState;
                }
                break;
            case 4:
                if (timer.milliseconds() > offTimeMs) {
                    switch (motif) {
                        case DecodeTag_Obelisk_PPG:
                        case DecodeTag_Obelisk_GPP:
                            SetColor(IndicatorColor.INDICATOR_INDIGO);
                            break;
                        case DecodeTag_Obelisk_PGP:
                            SetColor(IndicatorColor.INDICATOR_GREEN);
                            break;
                        default:
                            SetColor(IndicatorColor.INDICATOR_WHITE);
                            break;
                    }
                    timer.reset();
                    ++idleState;
                }
                break;
            case 6:
                if (timer.milliseconds() > offTimeMs) {
                    switch (motif) {
                        case DecodeTag_Obelisk_GPP:
                        case DecodeTag_Obelisk_PGP:
                            SetColor(IndicatorColor.INDICATOR_INDIGO);
                            break;
                        case DecodeTag_Obelisk_PPG:
                            SetColor(IndicatorColor.INDICATOR_GREEN);
                            break;
                        default:
                            SetColor(IndicatorColor.INDICATOR_WHITE);
                            break;
                    }
                    timer.reset();
                    ++idleState;
                }
                break;
            case 1:
            case 3:
            case 5:
            case 7:
                if (timer.milliseconds() > onTimeIdle) {
                    SetColor(IndicatorColor.INDICATOR_OFF);
                    timer.reset();
                    if (++idleState > 7) {
                        idleState = 0;
                    }
                }
                break;
            default:
                timer.reset();
                idleState = 0;
                break;
        }
        /*if (timer.milliseconds() > 10) {
            double last = led.getPosition();
            if (last >= .74) {
                up = false;
            } else if (last <= .28) {
                up = true;
            }
            double step;
            if (up) {
                step = .001;
            } else {
                step = -.001;
            }
            led.setPosition(last + step);
        }*/
    }

    private void SetColor(IndicatorColor color) {
        led.setPosition(GetValueForColor(color));
    }

    private static double GetValueForColor(IndicatorColor color) {
        switch (color) {
            case INDICATOR_RED:
                return 0.277;
            case INDICATOR_ORANGE:
                return 0.333;
            case INDICATOR_YELLOW:
                return 0.388;
            case INDICATOR_SAGE:
                return 0.444;
            case INDICATOR_GREEN:
                return 0.5;
            case INDICATOR_AZURE:
                return 0.555;
            case INDICATOR_BLUE:
                return 0.611;
            case INDICATOR_INDIGO:
                return 0.666;
            case INDICATOR_VIOLET:
                return 0.722;
            case INDICATOR_WHITE:
                return 1;
            case INDICATOR_OFF:
            default:
                return 0;
        }
    }

    public enum IndicatorColor {
        INDICATOR_OFF,
        INDICATOR_RED,
        INDICATOR_ORANGE,
        INDICATOR_YELLOW,
        INDICATOR_SAGE,
        INDICATOR_GREEN,
        INDICATOR_AZURE,
        INDICATOR_BLUE,
        INDICATOR_INDIGO,
        INDICATOR_VIOLET,
        INDICATOR_WHITE
    }
}
