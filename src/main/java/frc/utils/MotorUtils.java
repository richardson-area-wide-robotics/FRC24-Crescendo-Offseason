package frc.utils;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

public class MotorUtils {
    /**
     * Returns the speed of the motor
     */
    public static double getSpeed(CANSparkFlex motor){
        return motor.get();
    }

    /**
     * Returns output current of the motor
     */
    public static double getCurrent(CANSparkFlex motor){
        return motor.getOutputCurrent();
    }

    /**
     * Returns output current of the motor
     */
    public static double getCurrent(CANSparkMax motor){
        return motor.getOutputCurrent();
    }


    public static void setSpeed(CANSparkMax motor, double speed) {
        motor.set(speed);
    }

    public static void stopMotor(CANSparkFlex motor){
        motor.stopMotor();
    }
    public static void stopMotor(CANSparkMax motor){
        motor.stopMotor();
    }

}
