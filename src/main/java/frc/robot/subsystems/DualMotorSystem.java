package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.constants.MotorConfigs;

public class DualMotorSystem extends SubsystemBase {
    final private String CAN_BUS = "rio";
    final private String MOTOR_TYPE = "Falcon500";
    private TalonFX m_Ian;
    private TalonFX m_Aar;
    private VoltageOut voltageOut;
    private double voltageInput;

    public DualMotorSystem() {
        m_Ian = new TalonFX(1, CAN_BUS);
        m_Aar = new TalonFX(2, CAN_BUS);
        voltageInput = 0.5;
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration()
                .withCurrentLimits(MotorConfigs.getCurrentLimitConfig(MOTOR_TYPE))
                .withMotorOutput(
                        MotorConfigs.getMotorOutputConfigs(NeutralModeValue.Coast, InvertedValue.Clockwise_Positive))
                .withFeedback(MotorConfigs.getFeedbackConfigs(1 / 1));

        Follower followerConfig = new Follower(m_Ian.getDeviceID(), true); // CHANGE DEPENDING ON GEARBOX
        m_Ian.setControl(followerConfig);

        m_Ian.getConfigurator().apply(motorConfigs);
        m_Aar.getConfigurator().apply(motorConfigs);
    }
}
