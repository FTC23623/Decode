package org.firstinspires.ftc.teamcode.objects;

import java.util.ArrayDeque;
import java.util.Deque;

public class SampleAverage {
    private final Deque<Double> mSamples;
    private final int mSamplesToAvg;

    SampleAverage(int samplesToAvg) {
        if (samplesToAvg <= 0) {
            mSamplesToAvg = 1;
        } else {
            mSamplesToAvg = samplesToAvg;
        }
        mSamples = new ArrayDeque<>();
    }

    double GetAverage() {
        if (mSamples.isEmpty()) {
            return 0;
        } else {
            double sum = 0;
            for (double sample : mSamples) {
                sum += sample;
            }
            return sum / mSamples.size();
        }
    }

    void AddSample(double sample) {
        mSamples.addLast(sample);
        if (mSamples.size() > mSamplesToAvg) {
            mSamples.removeFirst();
        }
    }

    boolean Ready() {
        return mSamples.size() >= mSamplesToAvg;
    }
}
