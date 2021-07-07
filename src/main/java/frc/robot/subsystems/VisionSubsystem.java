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
    private double tx, tv, ta, txb, tvb, tab;
    private ArrayList<Double> m_targetList;
    private final int MAXENTRIES = 50;

    public VisionSubsystem() {
        m_frontLimeLightTable = NetworkTableInstance.getDefault().getTable("limelight-front");
        m_backLimeLightTable = NetworkTableInstance.getDefault().getTable("limelight-back");
        m_targetList = new ArrayList<Double>(MAXENTRIES);

        Shuffleboard.getTab("Vision").add("TX", tx).getEntry();
        Shuffleboard.getTab("Vision").add("TA", ta).getEntry();
        Shuffleboard.getTab("Vision").add("Target", tv).getEntry();
    }

    @Override
    public void periodic() {
        tx = m_frontLimeLightTable.getEntry("tx").getDouble(0);
        ta = m_frontLimeLightTable.getEntry("ta").getDouble(0);
        tv = m_frontLimeLightTable.getEntry("tv").getDouble(0);

        txb = m_backLimeLightTable.getEntry("txb").getDouble(0);
        tab = m_backLimeLightTable.getEntry("tab").getDouble(0);
        tvb = m_backLimeLightTable.getEntry("tvb").getDouble(0);

        if (m_targetList.size() >= MAXENTRIES) {
            m_targetList.remove(0);
        } else {
            m_targetList.add(ta);
        }
    }

    public double getTx() {
        return tx;
    }

    public double getTa() {
        double sum = 0;
        for (Double num : m_targetList) {
            sum += num.doubleValue();
        }
        return sum / m_targetList.size();
    }

    public boolean hasValidTarget() {
        return (tv == 1.0);
    }
}
