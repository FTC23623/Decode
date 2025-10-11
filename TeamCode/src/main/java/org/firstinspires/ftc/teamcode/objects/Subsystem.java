package org.firstinspires.ftc.teamcode.objects;

public interface Subsystem {
    // called once after the opmode inits and before it opmode starts
    abstract public void Init();
    // called every process loop
    abstract public void Process();
    // called every process loop
    abstract public void HandleUserInput();
}
