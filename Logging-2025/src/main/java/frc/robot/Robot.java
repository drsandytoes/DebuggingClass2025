package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import dev.doglog.DogLog;
import static edu.wpi.first.units.Units.Meters;

public class Robot extends TimedRobot {

    private Joystick driver = new Joystick(0);
    private Drivetrain drivetrain = new Drivetrain();
    private FakeSensor sensor = new FakeSensor();

    @Override
    public void teleopPeriodic() {
        double forward = -driver.getRawAxis(1);

        // Drive normally (works fine)
        drivetrain.arcadeDrive(forward, 0.0);

        // Get raw distance from sensor
        double rawDistance = sensor.getDistance();

        // Incorrectly filtered distance (this is the bug!)
        double filteredDistance = MathUtils.filter(rawDistance);

        DogLog.log("Sensor/RawDistance", rawDistance, Meters);
        DogLog.log("Sensor/FilteredDistance", filteredDistance, Meters);
    }
}
