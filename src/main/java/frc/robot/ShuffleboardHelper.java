/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.Tools.DataTools.Toggle;
import frc.robot.Tools.DataTools.Toggleable;
import frc.robot.subsystems.ControlPanelController;

/**
 * A set of <i><b>handy</i></b> tools for using the Shuffleboard!
 * 
 */
public class ShuffleboardHelper {
    /**
     * This method adds a list of CANSparkMaxes (as a Map to give them names) to a layout that will automatically update their values
     * @param listName The name of the layout/list to put the motors in (for organization, it is in its own layout)
     * @param devices A map of devices and their names to put in the layout. Use <b>"Map.of()"</b> to create this.
     */
    public static void addSparkMaxLayout(String tab, String listName, Map<CANSparkMax,String> devices) {
        ShuffleboardLayout layout = Shuffleboard.getTab(tab).getLayout(listName, BuiltInLayouts.kList)
            .withSize(4, 4).withPosition(0, 0);
        devices.forEach((sparkMax, name)->{
            CANEncoder encoder = sparkMax.getEncoder();
            layout.addNumber(name+": Position", encoder::getPosition);
            layout.addNumber(name+": Velocity", encoder::getVelocity);
            layout.addNumber(name+": Voltage", sparkMax::getBusVoltage);
            layout.addNumber(name+": Current", sparkMax::getOutputCurrent);
        });
    }

    public static void addColorSensor(String name, ControlPanelController controller) {

        ShuffleboardLayout layout = Shuffleboard.getTab("ShuffleboardHelper").getLayout(name, BuiltInLayouts.kList)
            .withSize(4, 7).withPosition(15, 0);//.withProperties(Map.of("Label position", "HIDDEN"));
        
        InstantCommand setRedColor = new InstantCommand(()->{
            ControlPanelConstants.kRedTarget = controller.getColor();
        },controller);
        InstantCommand setGreenColor = new InstantCommand(()->{
            ControlPanelConstants.kGreenTarget = controller.getColor();
        },controller);
        InstantCommand setBlueColor = new InstantCommand(()->{
            ControlPanelConstants.kBlueTarget = controller.getColor();
        },controller);
        InstantCommand setYellowColor = new InstantCommand(()->{
            ControlPanelConstants.kYellowTarget = controller.getColor();
        },controller);
        InstantCommand setValues = new InstantCommand(()->{
            controller.setupColors();
        },controller);
        setRedColor.setName("Set Red Color");
        setGreenColor.setName("Set Green Color");
        setBlueColor.setName("Set Blue Color");
        setYellowColor.setName("Set Yellow Color");
        setValues.setName("Setup Colors");

        layout.add(setRedColor);
        layout.add(setGreenColor);
        layout.add(setBlueColor);
        layout.add(setYellowColor);
        layout.add(setValues);
    }

    public static void AddToggle(String listName, String name, Toggleable<Integer> toggleable) {
        ShuffleboardLayout layout = Shuffleboard.getTab("ShuffleboardHelper").getLayout(listName, BuiltInLayouts.kList)
            .withSize(4, 4).withPosition(20, 0);
        toggleable.init();
        InstantCommand c = new InstantCommand(toggleable::toggle);
        c.setName(name);
        layout.add(c);
    }
    public static void AddToggle(String listName, String name, Consumer<Integer> output, Toggle<Integer> toggle) {
        ShuffleboardLayout layout = Shuffleboard.getTab("ShuffleboardHelper").getLayout(listName, BuiltInLayouts.kList)
            .withSize(4, 4).withPosition(20, 0);
        InstantCommand c = new InstantCommand(()->{
            output.accept(toggle.swap());
        });
        c.setName(name);
        layout.add(c);
    }

    public static void AddOutput(Subsystem subsystem, String name, int min, int max, DoubleConsumer output, Sendable... commands) {
        ShuffleboardLayout layout = Shuffleboard.getTab("ShuffleboardHelper").getLayout(name, BuiltInLayouts.kList).withSize(3, 5);
        NetworkTableEntry position = layout.addPersistent("Output", 0.0).withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min",min,"max",max)).withSize(5, 2).withPosition(25, 0).getEntry();
        InstantCommand setOutput = new InstantCommand(()->{
            //System.out.println("Value is now "+position.getDouble(0.0));
            output.accept(position.getDouble(0.0));
        }, subsystem);
        setOutput.setName("Set Output");
        layout.add(setOutput);
        InstantCommand zero = new InstantCommand(()->{
            output.accept(0);
            System.out.println("Zeroed the output.");
            //position.setNumber(0);
        }, subsystem);
        zero.setName("Zero Output");
        layout.add(zero);
        for(Sendable c : commands) {
            layout.add(c);
        }
    }
}
