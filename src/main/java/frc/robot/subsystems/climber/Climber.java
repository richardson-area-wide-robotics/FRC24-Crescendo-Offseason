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
import frc.utils.MotorUtils;

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
     * @param motor the motor to configure
     * @param isLeftSide true if configuring the left motor, false if right
     */
    private void climberConfig(CANSparkMax motor, boolean isLeftSide) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(ClimberConstants.kClimberIdleMode);
        motor.setSmartCurrentLimit(ClimberConstants.kClimberCurrentLimit);
        motor.enableSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kClimberForwardSoftLimitEnabled);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.kClimberReverseSoftLimitEnabled);
        motor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kClimberForwardSoftLimit);
        motor.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.kClimberReverseSoftLimit);
        motor.burnFlash();

        if (isLeftSide) {
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

    /**
     * Levels the robot (to roll = 0 degrees) using the climber.
     *
     * @param rollSupplier a supplier providing the current roll angle
     */
    public void level(Supplier<Measure<Angle>> rollSupplier) {
        while (Math.abs(rollSupplier.get().in(Degrees)) > ClimberConstants.kRollTolerance.in(Degrees)) {
            boolean useLeft = rollSupplier.get().in(Degrees) > 0;
            setMotorSpeed(useLeft, Math.signum(rollSupplier.get().in(Degrees)) * ClimberConstants.kLevelSpeed);
        }
    }

    /**
     * Sets the speed of the climber motors.
     *
     * @param useLeft true to control the left motor, false for right
     * @param speed the speed to set the motor
     */
    private void setMotorSpeed(boolean useLeft, double speed) {
        if (useLeft) {
            m_climberLeftMotor.set(speed);
        } else {
            m_climberRightMotor.set(speed);
        }
    }

    /**
     * Sets the direction of both climber motors.
     *
     * @param direction the direction to move the motors (UP or DOWN)
     */
    public void setDirection(ClimberDirection direction) {
        double speed = direction == ClimberDirection.UP ? ClimberConstants.kClimbSpeed : -ClimberConstants.kClimbSpeed;
        setSpeed(speed);
    }

    /**
     * Sets the speed of both climber motors.
     *
     * @param speed the speed to set both motors
     */
    public void setSpeed(double speed) {
        m_climberLeftMotor.set(speed);
        m_climberRightMotor.set(speed);
    }

    /**
     * Stops both climber motors.
     */
    public void stop() {
        m_climberLeftMotor.stopMotor();
        m_climberRightMotor.stopMotor();
    }

    /**
     * Sets the default command to stop the climber motors.
     */
    private void setDefaultCommand() {
        super.setDefaultCommand(Commands.run(this::stop, this));
    }

    /**
     * Command to climb up.
     *
     * @return the climb up command
     */
    public Command climbUp() {
        return Commands.run(() -> setDirection(ClimberDirection.UP), this);
    }

    /**
     * Command to climb down.
     *
     * @return the climb down command
     */
    public Command climbDown() {
        return Commands.run(() -> setDirection(ClimberDirection.DOWN), this);
    }
}
