package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class Elevator extends SubsystemBase{
    private TalonFX motor;
    private SysIdRoutine routine;

    private ArmFeedforward armFeedforward;

    private LoggedTunableNumber armKs = new LoggedTunableNumber("armKs", 0.0);
    private LoggedTunableNumber armKg = new LoggedTunableNumber("armKg", 0.0);
    private LoggedTunableNumber armKv = new LoggedTunableNumber("armKv", 0.0);
    private LoggedTunableNumber armP = new LoggedTunableNumber("armP", 0.0);
    private LoggedTunableNumber armD = new LoggedTunableNumber("armd", 0.0);

    private LoggedTunableNumber maxV = new LoggedTunableNumber("maxV", 0.0);
    private LoggedTunableNumber maxA = new LoggedTunableNumber("maxA", 0.0);

    private State currentState = new State();
    private State targetState = new State();

    private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new Constraints(maxV.get(), maxA.get()));

    private CommandXboxController controller;

    public Elevator(CommandXboxController controller) {
        this.controller = controller;

        armFeedforward = new ArmFeedforward(0, 0, 0);

        SysIdRoutine.Mechanism sysIdMech = new SysIdRoutine.Mechanism(
            this::setVoltage,
            this::motorSysIdLog,
            this
        );

        routine = new SysIdRoutine(new Config(Volts.of(1).per(Second), Volts.of(1), Seconds.of(3)), sysIdMech);
        currentState.position = getPosition().in(Inches);
    }

    private void motorSysIdLog(SysIdRoutineLog log) {
        log.motor("armMotor")
            .voltage(Volts.of(motor.getAppliedOutput() * motor.getBusVoltage()))
            .angularPosition(getPosition())
            .angularVelocity(getVelocity());
    }

    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    public void setTargetPosition(Distance position) {
        targetPosition = Inches.of(MathUtil.clamp(position.in(Inches), -90, 90));
    }

    public Distance getPosition() {
        return Meters.of((2 * Math.PI * WHEEL_RADIUS.in(Meters) * motor.getPosition.in(Rotations)) / GEARING);
        // return Inches.of(MathUtil.angleModulus(Inches.of(motor.getEncoder().getPosition()).in(Inches)));
    }

    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(motor.getEncoder().getVelocity());
    }  
    
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    private Distance targetPosition = getPosition();

    @Override
    public void periodic() {        
        Logger.recordOutput("Arm Position", getPosition().in(Degrees));
        Logger.recordOutput("Arm Velocity", getVelocity().in(DegreesPerSecond));
        Logger.recordOutput("Voltage", motor.getAppliedOutput() * motor.getBusVoltage());
        Logger.recordOutput("Setpoint", targetPosition.getDegrees());

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            armFeedforward = new ArmFeedforward(armKs.get(), armKg.get(), armKv.get());
        }, armKg, armKs, armKv);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            SparkMaxConfig motorConfig = new SparkMaxConfig();
            motorConfig.apply(
                new EncoderConfig()
                    .positionConversionFactor(1.0/25)
                    .velocityConversionFactor(1.0/25/60)
            ).apply(
                new ClosedLoopConfig()
                    .pid(armP.get(), 0, armD.get())
            );

            System.out.println("PID");
    
            motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }, armP, armD);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            trapezoidProfile = new TrapezoidProfile(new Constraints(maxV.get(), maxA.get()));
        }, maxV, maxA);

        // double x = MathUtil.applyDeadband(controller.pov(), 0.1);
        // double y = -MathUtil.applyDeadband(controller.getLeftY(), 0.1);
        // if (x != 0 || y != 0) {
        //     latestPosition = new Rotation2d(x, y);
        // }

        // Logger.recordOutput("Controller position", latestPosition.getDegrees());

        if (RobotState.isDisabled()) {
            setTargetPosition(new Rotation2d(getPosition()));
        }

        targetState = new State(targetPosition.getRotations(), 0);

        currentState = trapezoidProfile.calculate(0.02, currentState, targetState);

        Logger.recordOutput("TrapozoidNewPositon", currentState.position*360);
        Logger.recordOutput("TrapozoidTargetPoisiton", targetState.position*360);

        motor.getClosedLoopController().setReference(currentState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, armFeedforward.calculate(currentState.position, currentState.velocity));
    }

    public Command setTargetPositionCommand(Rotation2d position) {
        return runOnce(() -> setTargetPosition(position));
    }
}
