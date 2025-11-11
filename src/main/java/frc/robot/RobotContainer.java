// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot; 

import frc.robot.commands.Autos;
import frc.robot.subsystems.DualMotorSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DualMotorSystem dual = new DualMotorSystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandPS5Controller driver = new CommandPS5Controller(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // L1 button: Bottom motor counterclockwise, upper motor at rest
        driver.L1().whileTrue(new RunCommand(() -> {
            System.out.println("L1 pressed!");
            dual.bottomCCWUpperRest(driver.getRightY());
        }, dual));

        // L2 button: Bottom motor clockwise, upper motor counterclockwise
        driver.L2().whileTrue(new RunCommand(() -> {
            System.out.println("L2 pressed!");
            dual.bottomCWUpperCCW(driver.getRightY());
        }, dual));

        // R1 button: Bottom motor at rest, upper motor counterclockwise
        driver.R1().whileTrue(new RunCommand(() -> {
            System.out.println("R1 pressed!");
            dual.bottomRestUpperCCW(driver.getRightY());
        }, dual));

        // Options button: Stop both motors
        driver.options().onTrue(new InstantCommand(() -> {
            System.out.println("Options pressed!");
            dual.stopMotor();
        }, dual));
    }

    public Command getAutonomousCommand() {
        return Autos.dualAuto(dual);
    }
}
