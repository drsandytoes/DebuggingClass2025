package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public class MathUtils {
    private static double previous = 0.0;
    private static DoubleSubscriber newValueWeightTunable = DogLog.tunable("Filter/newValueFraction", 0.2);

    public static double filter(double newValue) {
        double newValueWeight = newValueWeightTunable.get();
        double oldValueWeight = 1.0 - newValueWeight;
        previous = oldValueWeight * previous + newValueWeight * newValue;
        return previous;
    }
}
