/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

/**
 * A set of <b>handy</b> tools for using the Shuffleboard!
 */
public class ShuffleboardHelper {
    public void addLayout(String listName, Map<CANSparkMax,String> devices) {
        //Map.of(devices.keySet().toArray()[3], "New Motor");
        //add(Array<CANDevice.of(new CANDevice(0, null, null)));
        CANSparkMax c = new CANSparkMax(0,MotorType.kBrushed);
        ShuffleboardLayout layout = Shuffleboard.getTab("ShuffleboardHelper").getLayout(listName, BuiltInLayouts.kList).withSize(4, 4).withPosition(0, 0);
        
        //Map.ofEntries(c,"");
        //addLayout("New Tab", new CANSparkMax(0,MotorType.kBrushed), "", new CANSparkMax(0,MotorType.kBrushed), ""));
        NetworkTableEntry[] entries = new NetworkTableEntry[devices.size()];
        //devices.forEach((sparkMax, name) -> {});
        int i = 0;
        for (Map.Entry<CANSparkMax,String> sparkMax : devices.entrySet()) {
            entries[i] = layout.add(sparkMax.getValue()+": Current", 0).getEntry();
            i++;
        }
    //left_front_velocity = layout.add("Left Front Velocity", 0).getEntry();
    }
    class SparkMaxDebugger{
        CANSparkMax sparkMax;
        NetworkTableEntry velocityEntry;
        NetworkTableEntry voltageEntry;
        NetworkTableEntry currentEntry;
        NetworkTableEntry positionEntry;
        DoubleSupplier getVelocity;
        DoubleSupplier getVoltage;
        DoubleSupplier getCurrent;
        DoubleSupplier getPosition;
        public SparkMaxDebugger(CANSparkMax sparkMax, NetworkTableEntry velocityEntry, NetworkTableEntry voltageEntry,
        NetworkTableEntry currentEntry, NetworkTableEntry positionEntry, DoubleSupplier getVelocity, DoubleSupplier getVoltage,
        DoubleSupplier getCurrent, DoubleSupplier getPosition){
            this.sparkMax = sparkMax;
            this.velocityEntry = velocityEntry;
            this.voltageEntry = voltageEntry;
            this.currentEntry = currentEntry;
            this.positionEntry = positionEntry;
            getVelocity = sparkMax.getEncoder()::getVelocity;
        }
        public SparkMaxDebugger(CANSparkMax sparkMax){
            this.sparkMax = sparkMax;
            this.velocityEntry = velocityEntry;
            this.voltageEntry = voltageEntry;
            this.currentEntry = currentEntry;
            this.positionEntry = positionEntry;
            getVelocity = sparkMax.getEncoder()::getVelocity;
        }
    }
}

