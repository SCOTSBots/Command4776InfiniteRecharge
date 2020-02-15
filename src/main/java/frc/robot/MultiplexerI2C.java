/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * Add your docs here.
 */
public class MultiplexerI2C {
    final int kAddress = 0x70;
    I2C target;
    int port;
    int portAddress;
	public MultiplexerI2C(int multiplexerPort, int address) {
        target = new I2C(Port.kOnboard, kAddress);
        port = multiplexerPort;
        portAddress = address;
	}

	public boolean read(byte bVal, int i, ByteBuffer raw) {
        setPort(port);
        boolean failed = target.read(bVal, i, raw);
        return failed;
	}

	public boolean write(byte bVal, int data) {
        setPort(port);
        boolean failed = target.write(bVal, data);
        return failed;
	}

    void setPort(int i) {
        //if (i > 7) return;
        target.write(kAddress, 1 << i);
        target.close();
      }

}
