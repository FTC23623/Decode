package org.firstinspires.ftc.teamcode.objects;

public class LowPassFilter {
    private double gain; // Alpha (0 to 1)
    private double lastEstimate;
    private boolean initialized = false;

    public LowPassFilter(double gain) {
        this.gain = gain;
    }

    public void update(double input, double gain) {
        if (!initialized) {
            lastEstimate = input;
            initialized = true;
        }
        setGain(gain);
        // Formula: New = (Gain * Current) + ((1 - Gain) * Previous)
        lastEstimate = (gain * input) + ((1 - gain) * lastEstimate);
    }

    public double getValue() {
        return lastEstimate;
    }

    public void reset() {
        initialized = false;
    }

    public void setGain(double gain) {
        this.gain = gain;
    }
}