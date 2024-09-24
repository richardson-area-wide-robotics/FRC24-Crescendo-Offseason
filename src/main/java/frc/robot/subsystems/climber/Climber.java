package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Angle;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberDirection;
import java.util.function.Supplier;

public class Climber extends SubsystemBase {
    private final CANSparkMax m_climberLeftMotor;
    private final CANSparkMax m_climberRightMotor;

    private SparkPIDController m_climberLeftPIDController;
    private SparkPIDController m_climberRightPIDController;

    private RelativeEncoder m_climberLeftEncoder;
    private RelativeEncoder m_climberRightEncoder;

    public Climber() {
        m_climberLeftMotor = new CANSparkMax(ClimberConstants.kClimberLeftCANID, MotorType.kBrushless);
        m_climberRightMotor = new CANSparkMax(ClimberConstants.kClimberRightCANID, MotorType.kBrushless);

        climberConfig(m_climberLeftMotor, true);
        climberConfig(m_climberRightMotor, false);

        setDefaultCommand();
    }

    /**
     * Configures a CANSparkMax motor for the climber.
     *
     * @param motor          the motor to configure
     * @param climberLeftSide true if configuring the left motor, false if right
     */
    private void climberConfig(CANSparkMax motor, boolean climberLeftSide) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(ClimberConstants.kClimberIdleMode);
        motor.setSmartCurrentLimit(ClimberConstants.kClimberCurrentLimit);
        motor.enableSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kClimberForwardSoftLimitEnabled);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.kClimberReverseSoftLimitEnabled);
        motor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kClimberForwardSoftLimit);
        motor.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.kClimberReverseSoftLimit);
        motor.burnFlash();

        if (climberLeftSide) {
            motor.setInverted(ClimberConstants.kClimberLeftInverted);
            m_climberLeftEncoder = motor.getEncoder();
            m_climberLeftPIDController = motor.getPIDController();
            configurePIDController(m_climberLeftPIDController);
        } else {
            motor.setInverted(ClimberConstants.kClimberRightInverted);
            m_climberRightEncoder = motor.getEncoder();
            m_climberRightPIDController = motor.getPIDController();
            configurePIDController(m_climberRightPIDController);
        }
    }

    /**
     * Configures the PID controller for the climber motors.
     *
     * @param pidController the PID controller to configure
     */
    private void configurePIDController(SparkPIDController pidController) {
        pidController.setP(ClimberConstants.kClimberP);
        pidController.setI(ClimberConstants.kClimberI);
        pidController.setD(ClimberConstants.kClimberD);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Levels the robot (to roll = 0 degrees) using the climber.
     *
     * @param rollSupplier a supplier providing the current roll angle
     */
    public void level(Supplier<Measure<Angle>> rollSupplier) {
        boolean useLeft = rollSupplier.get().in(Degrees) > 0;
        while (Math.abs(rollSupplier.get().in(Degrees)) > ClimberConstants.kRollTolerance.in(Degrees)) {
            if (rollSupplier.get().in(Degrees) > 0) {
                setMotorSpeed(useLeft, ClimberConstants.kLevelSpeed);
            } else {
                setMotorSpeed(useLeft, -ClimberConstants.kLevelSpeed);
            }
        }
    }

    private void setMotorSpeed(boolean useLeft, double speed) {
        if (useLeft) {
            m_climberLeftMotor.set(speed);
        } else {
            m_climberRightMotor.set(speed);
        }
    }

    public void setLeftDirection(ClimberDirection direction) {
        setLeftSpeed(direction == ClimberDirection.UP ? ClimberConstants.kClimbSpeed : -ClimberConstants.kClimbSpeed);
    }

    public void setLeftSpeed(double speed) {
        m_climberLeftMotor.set(speed);
    }

    public void setRightDirection(ClimberDirection direction) {
        setRightSpeed(direction == ClimberDirection.UP ? ClimberConstants.kClimbSpeed : -ClimberConstants.kClimbSpeed);
    }

    public void setRightSpeed(double speed) {
        m_climberRightMotor.set(speed);
    }

    public void setDirection(ClimberDirection direction) {
        setSpeed(direction == ClimberDirection.UP ? ClimberConstants.kClimbSpeed : -ClimberConstants.kClimbSpeed);
    }

    public void setSpeed(double speed) {
        m_climberLeftMotor.set(speed);
        m_climberRightMotor.set(speed);
    }

    public void stop() {
        m_climberLeftMotor.stopMotor();
        m_climberRightMotor.stopMotor();
    }

    /**
     * Sets the default command for the climber to stop the climber motors.
     */
    private void setDefaultCommand() {
        super.setDefaultCommand(Commands.run(this::stop, this));
    }

    public Command climbUp() {
        return Commands.run(() -> setDirection(ClimberDirection.UP), this);
    }

    public Command climbDown() {
        return Commands.run(() -> setDirection(ClimberDirection.DOWN), this);
    }
}
