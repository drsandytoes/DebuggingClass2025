package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.RobotController;
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

public class Elevator extends SubsystemBase {
  private final double kGearRatio = 3.0;
  private final double kDrumDiameter = Units.inchesToMeters(1.0);
  private final double kRotationsPerMeter = kGearRatio / (Math.PI * kDrumDiameter);

  private final TalonFX m_motor = new TalonFX(0);

  private ControlMode m_controlMode = ControlMode.VOLTAGE;
  private double m_targetVoltage = 0.0;
  private double m_targetPosition = 0.0;

  private ProfiledPIDController m_controller;
  private ElevatorFeedforward m_feedforward;

  private DoubleSubscriber m_tunable_kP =
      DogLog.tunable(
          "kP",
          100.0,
          (double kP) -> {
            m_controller.setP(kP);
          });
  private DoubleSubscriber m_tunable_kI =
      DogLog.tunable(
          "kI",
          0.0,
          (double kI) -> {
            m_controller.setI(kI);
          });
  private DoubleSubscriber m_tunable_kD =
      DogLog.tunable(
          "kD",
          0.0,
          (double kD) -> {
            m_controller.setD(kD);
          });
  private DoubleSubscriber m_tunable_kS =
      DogLog.tunable(
          "kS",
          0.0,
          (double kS) -> {
            m_feedforward.setKs(kS);
          });
  private DoubleSubscriber m_tunable_kG =
      DogLog.tunable(
          "kG",
          1.1,
          (double kG) -> {
            m_feedforward.setKg(kG);
          });
  private DoubleSubscriber m_tunable_kV =
      DogLog.tunable(
          "kV",
          4.6,
          (double kV) -> {
            m_feedforward.setKv(kV);
          });

  enum ControlMode {
    VOLTAGE,
    POSITION,
  }

  public Elevator() {
    SmartDashboard.putData("Elevator Viz", m_simViz);

    double kP = m_tunable_kP.get();
    double kI = m_tunable_kI.get();
    double kD = m_tunable_kD.get();
    double kG = m_tunable_kG.get();
    double kV = m_tunable_kV.get();
    double kS = m_tunable_kS.get();

    m_controller = new ProfiledPIDController(kP, kI, kD, new Constraints(1.0, 2.0));
    m_feedforward = new ElevatorFeedforward(kS, kG, kV);
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

          m_controller.reset(getCurrentPosition());
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
          State setpoint = m_controller.getSetpoint();
          m_motor.setVoltage(
              m_controller.calculate(getCurrentPosition(), m_targetPosition)
                  + m_feedforward.calculate(setpoint.velocity));

          DogLog.log("Elevator/Profile Position (m)", setpoint.position);
          DogLog.log("Elevator/Profile Velocity (m per s)", setpoint.velocity);

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
