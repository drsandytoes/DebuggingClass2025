package frc.robot;

import java.util.Random;

public class SensorManager {
    private double[] distances = { 1.0, 1.5, 2.0 };
    private double divisor = 1.0;

    public double getAverageDistance() {
        Random rand = new Random();
        int randomIndex = rand.nextInt(4);
        System.out.println("Random index: " + randomIndex);

        double val = distances[randomIndex]; 
        return MathUtils.divide(MathUtils.average(val, distances[1]), divisor);
    }

    public void setDivisor(double newValue) {
        divisor = newValue;
    }
}
