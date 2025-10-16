package org.firstinspires.ftc.teamcode.objects;

public interface Subsystem {
    // called repeatedly until all systems are initialized
    abstract public boolean Init();
    // called every process loop
    abstract public void Process();
    // called every process loop
    abstract public void HandleUserInput();
}
