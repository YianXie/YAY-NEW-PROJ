package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.constants.MotorConfigs;

public class DualMotorSystem extends SubsystemBase {
    final private String CAN_BUS = "rio";
    final private String MOTOR_TYPE = "Falcon500";
    private TalonFX m_Top;
    private TalonFX m_Bottom;

    /** the state of the bottom motor */
    private VoltageOut voltageOut;
    private double voltageInput;

    /**
     * The constructor
     */
    public DualMotorSystem() {
        this.m_Top = new TalonFX(1, CAN_BUS);
        this.m_Bottom = new TalonFX(2, CAN_BUS);
        this.voltageInput = 0.5;
        this.voltageOut = new VoltageOut(0);
        configureMotors();
    }

    /**
     * Configure the motors
     */
    private void configureMotors() {
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(MotorConfigs.getCurrentLimitConfig(MOTOR_TYPE))
                .withMotorOutput(
                        MotorConfigs.getMotorOutputConfigs(NeutralModeValue.Coast, InvertedValue.Clockwise_Positive))
                .withFeedback(MotorConfigs.getFeedbackConfigs(1 / 1));

        m_Top.getConfigurator().apply(motorConfigs);
        m_Bottom.getConfigurator().apply(motorConfigs);
    }

    /**
     * Set the control of a specific motor
     * 
     * @param motor
     *            - the motor you are trying to modify
     * @param req
     *            - the control request
     */
    private void setControl(TalonFX motor, ControlRequest req) {
        if (motor.isAlive()) {
            motor.setControl(req);
        }
    }

    /**
     * Get the state of a motor (either clockwise, counterclockwise, or at rest) as
     * an int
     * 
     * @param motor
     *            - the motor you are getting the state of
     * @return the state of the motor: -1 = clockwise, 1 = counterclockwise, 0 = at
     *         rest
     */
    public int getMotorState(TalonFX motor) {
        double velocity = motor.getVelocity().getValueAsDouble();

        // Use 0.01 to avoid noise issue
        if (velocity > 0.01) {
            // Clockwise
            return -1;
        } else if (velocity < -0.01) {
            // Counterclockwise
            return 1;
        }

        // At rest
        return 0;
    }

    /**
     * Increase the voltage of a specific motor
     * 
     * @param motor
     *            - the motor you are trying to increase the voltage
     * @param input
     *            - the amount of voltage you are trying to increase (this is
     *            always positive)
     */
    public void voltageUp(TalonFX motor, double input) {
        if (input < 0)
            throw new Error("The input must be positive");
        setControl(motor, voltageOut.withOutput(input));
    }

    /**
     * Increase the bottom motor's voltage with the defined `voltageInput` variable
     */
    public void voltageUp() {
        setControl(m_Bottom, voltageOut.withOutput(voltageInput));
    }

    /**
     * Decrease the voltage of a specific motor
     * 
     * @param motor
     *            - the motor you are trying to decrease the voltage
     * @param input
     *            - the amount of voltage you are trying to decrease (this is
     *            always positive)
     */
    public void voltageDown(TalonFX motor, double input) {
        if (input < 0)
            throw new Error("The input must be positive");
        setControl(motor, voltageOut.withOutput(-input));
    }

    /**
     * Decrease the bottom motor's voltage with the defined `voltageInput` variable
     */
    public void voltageDown() {
        setControl(m_Bottom, voltageOut.withOutput(-voltageInput));
    }

    /**
     * Stop the 2 motors
     */
    public void stopMotor() {
        setControl(m_Top, voltageOut.withOutput(0));
        setControl(m_Bottom, voltageOut.withOutput(0));
    }

    /**
     * Get the velocity of a specified motor
     * 
     * @param motor
     *            - the motor you are getting the velocity
     * @return the velocity of that motor
     */
    public double getVelocity(TalonFX motor) {
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * Get the velocity of the bottom motor
     * 
     * @return the velocity of the bottom motor
     */
    public double getVelocity() {
        return m_Bottom.getVelocity().getValueAsDouble();
    }

    public void driveOpenLoop(double bottomVolts, boolean hasTopOverride) {
        double topVolts;

        if (hasTopOverride) {
            // Top spins by itself
            topVolts = this.voltageInput;
        } else {
            if (bottomVolts > 0) { // bottom CW
                topVolts = -this.voltageInput; // top CCW (1:1)
            } else if (bottomVolts < 0) { // bottom CCW
                topVolts = 0; // top rests
            } else { // bottom rests
                topVolts = 0;
            }
        }

        setControl(m_Bottom, voltageOut.withOutput(bottomVolts));
        setControl(m_Top, voltageOut.withOutput(topVolts));
    }
}
