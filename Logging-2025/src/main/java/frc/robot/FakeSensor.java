package frc.robot;

public class FakeSensor {
    private double value = 50.0;

    public double getDistance() {
        // simulate a drifting sensor reading
        value += (Math.random() - 0.5) * 5;
        return value;
    }
}
