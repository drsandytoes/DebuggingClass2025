package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;


public class Robot extends TimedRobot {
    private Drivetrain drivetrain;
    private SensorManager sensors;
    private CommandJoystick joystick = new CommandJoystick(0);
    private double sensorDivisor;

    public void robotInit() {
        System.out.println("Robot initializing...");
        drivetrain = new Drivetrain();

        configureBindings();
      }

    public void robotPeriodic() {
        System.out.println("Running robotPeriodic...");
        CommandScheduler.getInstance().run();
        // Example of calling into other classes
        double distance = sensors.getAverageDistance();
        System.out.println("Average distance: " + distance);

        drivetrain.arcadeDrive(0.5, 0.1);
    }
    
    private void configureBindings() {
      if (joystick != null) {
        joystick.button(1).whileTrue(Commands.run(() -> {
          sensors.setDivisor(sensorDivisor);
        }).ignoringDisable(true));
      }
    }
  }


