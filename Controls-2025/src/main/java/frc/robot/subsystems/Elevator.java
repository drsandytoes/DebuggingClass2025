package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.function.BooleanSupplier;

public class Elevator extends SubsystemBase {
  private final double kGearRatio = 3.0;
  private final double kDrumDiameter = Units.inchesToMeters(1.0);
  private final double kRotationsPerMeter = kGearRatio / (Math.PI * kDrumDiameter);

  private final TalonFX m_motor = new TalonFX(0);

  private ControlMode m_controlMode = ControlMode.VOLTAGE;
  private double m_targetVoltage = 0.0;
  private double m_targetPosition = 0.0;

  private DoubleSubscriber m_tunable_kP = DogLog.tunable("kP", 100.0);
  private DoubleSubscriber m_tunable_kD = DogLog.tunable("kD", 0.0);
  private DoubleSubscriber m_tunable_kI = DogLog.tunable("kI", 0.0);
  private DoubleSubscriber m_tunable_kG = DogLog.tunable("kG", 0.0);
  private DoubleSubscriber m_tunable_kV = DogLog.tunable("kV", 0.0);
  private BooleanSupplier m_tunable_trapezoidalProfile =
      DogLog.tunable("useTrapezoidalProfile", true);
  private BooleanSupplier m_tunable_limitIZone = DogLog.tunable("limitIZone", true);

  private final Timer m_timer = new Timer();
  private double m_lastError = 0.0;
  private double m_iError = 0.0;

  private final Timer m_profileTimer = new Timer();
  private final TrapezoidProfile m_profile = new TrapezoidProfile(new Constraints(1.0, 2.0));
  private State m_startState = new State();
  private State m_goalState = new State();

  enum ControlMode {
    VOLTAGE,
    POSITION,
  }

  public Elevator() {
    SmartDashboard.putData("Elevator Viz", m_simViz);
  }

  public Command setVoltageCommand(double voltage) {
    return runOnce(
        () -> {
          m_controlMode = ControlMode.VOLTAGE;
          m_targetVoltage = voltage;
        });
  }

  public Command setPositionCommand(double setpoint) {
    return runOnce(
        () -> {
          m_controlMode = ControlMode.POSITION;
          m_targetPosition = setpoint;

          m_timer.restart();
          m_lastError = m_targetPosition - getCurrentPosition();
          m_iError = 0.0;

          m_profileTimer.restart();
          m_startState = new State(getCurrentPosition(), getCurrentVelocity());
          m_goalState = new State(m_targetPosition, 0.0);
        });
  }

  public double getCurrentPosition() {
    return m_motor.getPosition().getValueAsDouble() / kRotationsPerMeter;
  }

  public double getCurrentVelocity() {
    return m_motor.getVelocity().getValueAsDouble() / kRotationsPerMeter;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    final double currentPosition = getCurrentPosition();

    switch (m_controlMode) {
      case VOLTAGE:
        {
          m_motor.setVoltage(m_targetVoltage);
          break;
        }
      case POSITION:
        {

          // Sample profile
          final State profileState =
              m_profile.calculate(m_profileTimer.get(), m_startState, m_goalState);
          DogLog.log("Elevator/Profile Position (m)", profileState.position);
          DogLog.log("Elevator/Profile Velocity (m per s)", profileState.velocity);

          // Target is either profile position (when using a profile) or the ultimate
          // target position
          double error =
              (m_tunable_trapezoidalProfile.getAsBoolean()
                      ? profileState.position
                      : m_targetPosition)
                  - currentPosition;
          double dError = (error - m_lastError) / m_timer.get();
          m_iError = m_iError + error * m_timer.get();

          // Reset integral if position error is greater than |kIZone|
          if (m_tunable_limitIZone.getAsBoolean()) {
            double kIZone = 0.01;
            if (Math.abs(error) > kIZone) {
              m_iError = 0.0;
            }
          }

          double kP = m_tunable_kP.get(); // volts per meter
          double kI = m_tunable_kI.get(); // volts per meter seconds
          double kD = m_tunable_kD.get(); // volts per m/s
          double kV = m_tunable_kV.get(); // volts per meter per second
          double kG = m_tunable_kG.get(); // volts

          if (!m_tunable_trapezoidalProfile.getAsBoolean()) {
            // Force kV to 0 if we're not using a profile
            kV = 0.0;
          }

          m_motor.setVoltage(
              kP * error + kI * m_iError + kD * dError + kV * profileState.velocity + kG);
          m_timer.restart();
          m_lastError = error;

          break;
        }
    }

    DogLog.log("Elevator/Control Mode", m_controlMode);
    DogLog.log("Elevator/Duty Cycle", m_motor.get());
    DogLog.log("Elevator/Current Position (m)", currentPosition);
    DogLog.log("Elevator/Target Position (m)", m_targetPosition);
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1), kGearRatio, 20.0, 0.5 * kDrumDiameter, 0.0, 2.0, true, 1.0);

  // Visualization
  private final double kBottomOffset = 20.0;
  private final double kPixelsPerMeter = 100.0;
  private final Mechanism2d m_simViz = new Mechanism2d(200, 300);
  private final MechanismRoot2d m_zRoot = m_simViz.getRoot("Z Root", 100, kBottomOffset);
  private final MechanismLigament2d m_z =
      m_zRoot.append(new MechanismLigament2d("Z", 220, 90, 10, new Color8Bit(Color.kBlue)));
  private final MechanismRoot2d m_carriageRoot = m_simViz.getRoot("Carriage", 0, 0);
  private final MechanismLigament2d m_carriage =
      m_carriageRoot.append(
          new MechanismLigament2d("Carriage", 20, 90, 10, new Color8Bit(Color.kYellow)));

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    final TalonFXSimState motorSim = m_motor.getSimState();
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_elevatorSim.setInputVoltage(m_motor.getMotorVoltage().getValueAsDouble());
    m_elevatorSim.update(Robot.periodSecs);

    // Convert motor rotations and motor rotations per second
    motorSim.setRawRotorPosition(m_elevatorSim.getPositionMeters() * kRotationsPerMeter);
    motorSim.setRotorVelocity(m_elevatorSim.getVelocityMetersPerSecond() * kRotationsPerMeter);

    // Update viz
    m_carriageRoot.setPosition(
        100, kBottomOffset + m_elevatorSim.getPositionMeters() * kPixelsPerMeter);
  }
  // --- END STUFF FOR SIMULATION ---
}
