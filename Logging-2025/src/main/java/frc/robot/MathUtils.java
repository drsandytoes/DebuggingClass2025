package frc.robot;

public class MathUtils {
    private static double previous = 0.0;

    public static double filter(double newValue) {
        previous = 0.8 * previous + 0.2 * previous; 
        return previous;
    }
}
