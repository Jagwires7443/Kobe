// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.ArrayList;

@SuppressWarnings("unused")
public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable m_frontLimeLightTable, m_backLimeLightTable;
    private double tx, tv;

    public VisionSubsystem() {
        m_frontLimeLightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
        m_backLimeLightTable = NetworkTableInstance.getDefault().getTable("limelight-back");

        Shuffleboard.getTab("Vision").add("TX", tx).getEntry();
        Shuffleboard.getTab("Vision").add("Target", tv).getEntry();
    }

    @Override
    public void periodic() {
        // tv Whether the limelight has any valid targets (0 or 1)
        // tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
        // ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
        // ta Target Area (0% of image to 100% of image)

        tx = m_frontLimeLightTable.getEntry("tx").getDouble(0);
        tv = m_frontLimeLightTable.getEntry("tv").getDouble(0);
    }

    public double getTx() {
        return tx;
    }

    public boolean hasValidTarget() {
        return (tv == 1.0);
    }
}
