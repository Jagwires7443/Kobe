// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDSubsystem extends SubsystemBase {

    private static Spark m_led = null;

    public LEDSubsystem(int port) {
        m_led = new Spark(port);
    }

    public void setColor(double color) {
        System.out.println("Changing LED Color to: " + color);
        m_led.set(color);
    }
}
