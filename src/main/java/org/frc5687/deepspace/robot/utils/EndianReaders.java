package org.frc5687.deepspace.robot.utils;

import edu.wpi.first.wpilibj.I2C;

import java.io.IOException;
import java.nio.ByteBuffer;

public class EndianReaders {
    public enum Endianness {
        LITTLE_ENDIAN,
        BIG_ENDIAN
    }


    /**
     * Read an unsigned byte from the I2C device
     */
    public static int readU8(I2C device, int reg){
        ByteBuffer buffer = ByteBuffer.allocate(1);
        boolean rc = device.read(reg, 1, buffer);
        int result = buffer.get(0);

//        if (verbose) {
//            System.out.println("I2C: Device " + i2caddr + " (0x" + Integer.toHexString(i2caddr) +
//                    ") returned " + result + " (0x" + Integer.toHexString(result) +
//                    ") from reg " + reg + " (0x" + Integer.toHexString(reg) + ")");
//        }

        return result; // & 0xFF;
    }

    /**
     * Read a signed byte from the I2C device
     */
    public static int readS8(I2C device, int reg){
        ByteBuffer buffer = ByteBuffer.allocate(1);
        boolean rc = device.read(reg,1, buffer);
        int result = buffer.get(0); // & 0x7F;
        if (result > 127){ result -= 256;}
        return result; // & 0xFF;
    }

    public static int readU16LE(I2C device,  int register){
        return EndianReaders.readU16(device,  register, Endianness.LITTLE_ENDIAN);
    }

    public static int readU16BE(I2C device, int register){
        return EndianReaders.readU16(device, register, Endianness.BIG_ENDIAN);
    }

    public static int readU16(I2C device, int register, Endianness endianness){
        int hi = EndianReaders.readU8(device, register);
        int lo = EndianReaders.readU8(device, register + 1);
        return ((endianness == Endianness.BIG_ENDIAN) ? (hi << 8) + lo : (lo << 8) + hi); // & 0xFFFF;
    }

    public static int readS16(I2C device, int register, Endianness endianness){
        int hi = 0, lo = 0;
        if (endianness == Endianness.BIG_ENDIAN) {
            hi = EndianReaders.readS8(device, register);
            lo = EndianReaders.readU8(device, register + 1);
        } else {
            lo = EndianReaders.readU8(device, register);
            hi = EndianReaders.readS8(device, register + 1);
        }
        return ((hi << 8) + lo); // & 0xFFFF;
    }

    public static int readS16LE(I2C device, int i2caddr, int register) {
        return EndianReaders.readS16(device, register, Endianness.LITTLE_ENDIAN);
    }

    public static int readS16BE(I2C device, int i2caddr, int register){
        return EndianReaders.readS16(device, register, Endianness.BIG_ENDIAN);
    }
}

