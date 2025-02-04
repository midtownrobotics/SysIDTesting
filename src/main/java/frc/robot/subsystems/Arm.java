package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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

public class Arm extends SubsystemBase{
    private SparkMax motor;
    private SysIdRoutine routine;

    private ArmFeedforward armFeedforward;

    private LoggedTunableNumber armKs = new LoggedTunableNumber("armKs", 0.0);
    private LoggedTunableNumber armKg = new LoggedTunableNumber("armKg", 0.0);
    private LoggedTunableNumber armKv = new LoggedTunableNumber("armKv", 0.0);
    private LoggedTunableNumber armP = new LoggedTunableNumber("armP", 0.0);

    private CommandXboxController controller;

    public Arm(CommandXboxController controller) {
        this.controller = controller;

        armFeedforward = new ArmFeedforward(0, 0, 0);

        motor = new SparkMax(31, MotorType.kBrushless);

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.apply(
            new EncoderConfig()
                .positionConversionFactor(1.0/25)
                .velocityConversionFactor(1.0/25/60)
        ).apply(
            new ClosedLoopConfig()
                .pid(0, 0, 0)
        );

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor.getEncoder().setPosition(-0.25);

        SysIdRoutine.Mechanism sysIdMech = new SysIdRoutine.Mechanism(
            this::setSpeed,
            this::motorSysIdLog,
            this
        );

        routine = new SysIdRoutine(new Config(), sysIdMech);
    }

    private void motorSysIdLog(SysIdRoutineLog log) {
        log.motor("armMotor")
            .voltage(Volts.of(motor.getAppliedOutput()))
            .angularPosition(getPosition())
            .angularVelocity(getVelocity());
    }

    public void setSpeed(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    public void setPosition(Rotation2d position) {
        motor.getClosedLoopController().setReference(position.getRotations(), ControlType.kPosition, ClosedLoopSlot.kSlot0, armFeedforward.calculate(position.getRotations(), getVelocity().in(RotationsPerSecond)));
    }

    public Angle getPosition() {
        return Rotations.of(motor.getEncoder().getPosition());
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

    private Rotation2d latestPosition = new Rotation2d();

    @Override
    public void periodic() {
        Logger.recordOutput("Arm Position", getPosition().in(Degrees));
        Logger.recordOutput("Arm Velocity", getVelocity().in(DegreesPerSecond));
        Logger.recordOutput("Voltage", motor.getAppliedOutput() * motor.getBusVoltage());

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
                    .pid(armP.get(), 0, 0)
            );

            System.out.println("kP changed");
    
            motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }, armP);

        double x = MathUtil.applyDeadband(controller.getLeftX(), 0.9);
        double y = MathUtil.applyDeadband(controller.getLeftY(), 0.9);
        if (x != 0 || y != 0) {
            latestPosition = new Rotation2d(x, y);
        }

        Logger.recordOutput("Controller position", latestPosition);

        setPosition(latestPosition);
    }
}
