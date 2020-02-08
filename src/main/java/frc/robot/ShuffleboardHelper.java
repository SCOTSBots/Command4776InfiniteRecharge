/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

/**
 * A set of <i><b>handy</i></b> tools for using the Shuffleboard!
 */
public class ShuffleboardHelper {
    /**
     * This method adds a list of CANSparkMaxes (as a Map to give them names) to a layout that will automatically update their values
     * @param listName The name of the layout/list to put the motors in (for organization, it is in its own layout)
     * @param devices A map of devices and their names to put in the layout. Use <b>"Map.of()"</b> to create this.
     */
    public static void addSparkMaxLayout(String listName, Map<CANSparkMax,String> devices) {
        ShuffleboardLayout layout = Shuffleboard.getTab("ShuffleboardHelper").getLayout(listName, BuiltInLayouts.kList).withSize(4, 4).withPosition(0, 0);
        devices.forEach((sparkMax, name)->{
            CANEncoder encoder = sparkMax.getEncoder();
            layout.addNumber(name+": Position", encoder::getPosition);
            layout.addNumber(name+": Velocity", encoder::getVelocity);
            layout.addNumber(name+": Voltage", sparkMax::getBusVoltage);
            layout.addNumber(name+": Current", sparkMax::getOutputCurrent);
        });
    }
}

