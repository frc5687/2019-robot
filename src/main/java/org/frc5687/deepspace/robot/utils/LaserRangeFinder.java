package org.frc5687.deepspace.robot.utils;

import edu.wpi.first.wpilibj.I2C;
import org.frc5687.deepspace.robot.Robot;

import static org.frc5687.deepspace.robot.utils.Helpers.hexStringToByteArray;

/**
 * https://github.com/OlivierLD/raspberry-coffee/blob/master/I2C.SPI/src/i2c/sensor/VL53L0X.java
 * 
 */

public class LaserRangeFinder extends OutliersProxy {
    private I2C _rangeFinder;
    public final static int VL53L0X_I2CADDR = 0x29;

    private final static int SYSRANGE_START = 0x00;
    private final static int SYSTEM_THRESH_HIGH = 0x0C;
    private final static int SYSTEM_THRESH_LOW = 0x0E;
    private final static int SYSTEM_SEQUENCE_CONFIG = 0x01;
    private final static int SYSTEM_RANGE_CONFIG = 0x09;
    private final static int SYSTEM_INTERMEASUREMENT_PERIOD = 0x04;
    private final static int SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
    private final static int GPIO_HV_MUX_ACTIVE_HIGH = 0x84;
    private final static int SYSTEM_INTERRUPT_CLEAR = 0x0B;
    private final static int RESULT_INTERRUPT_STATUS = 0x13;
    private final static int RESULT_RANGE_STATUS = 0x14;
    private final static int RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC;
    private final static int RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0;
    private final static int RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0;
    private final static int RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4;
    private final static int RESULT_PEAK_SIGNAL_RATE_REF = 0xB6;
    private final static int ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28;
    private final static int I2C_SLAVE_DEVICE_ADDRESS = 0x8A;
    private final static int MSRC_CONFIG_CONTROL = 0x60;
    private final static int PRE_RANGE_CONFIG_MIN_SNR = 0x27;
    private final static int PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56;
    private final static int PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57;
    private final static int PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64;
    private final static int FINAL_RANGE_CONFIG_MIN_SNR = 0x67;
    private final static int FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47;
    private final static int FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48;
    private final static int FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
    private final static int PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61;
    private final static int PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62;
    private final static int PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50;
    private final static int PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51;
    private final static int PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52;
    private final static int SYSTEM_HISTOGRAM_BIN = 0x81;
    private final static int HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33;
    private final static int HISTOGRAM_CONFIG_READOUT_CTRL = 0x55;
    private final static int FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70;
    private final static int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71;
    private final static int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72;
    private final static int CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20;
    private final static int MSRC_CONFIG_TIMEOUT_MACROP = 0x46;
    private final static int SOFT_RESET_GO2_SOFT_RESET_N = 0xBF;
    private final static int IDENTIFICATION_MODEL_ID = 0xC0;
    private final static int IDENTIFICATION_REVISION_ID = 0xC2;
    private final static int OSC_CALIBRATE_VAL = 0xF8;
    private final static int GLOBAL_CONFIG_VCSEL_WIDTH = 0x32;
    private final static int GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0;
    private final static int GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1;
    private final static int GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2;
    private final static int GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3;
    private final static int GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4;
    private final static int GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5;
    private final static int GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6;
    private final static int DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E;
    private final static int DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F;
    private final static int POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80;
    private final static int VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89;
    private final static int ALGO_PHASECAL_LIM = 0x30;
    private final static int ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30;
    private final static int VCSEL_PERIOD_PRE_RANGE = 0;
    private final static int VCSEL_PERIOD_FINAL_RANGE = 1;

    public LaserRangeFinder(Robot robot) {
        _rangeFinder = new I2C(I2C.Port.kOnboard, 0x52);
//		if (this.readU8(0xC0) != 0xEE || this.readU8(0xC1) != 0xAA || this.readU8(0xC2) != 0x10) {
        if (!_rangeFinder.verifySensor(0xC0, 3, hexStringToByteArray("EEAA10"))) {
            error("Unable to initialize RangeFinder.");
        }


    }
    private int readU8(int register){
        return EndianReaders.readU8(this._rangeFinder, VL53L0X_I2CADDR, register);
    }

    private int readS8(int register){
        return EndianReaders.readS8(this._rangeFinder, VL53L0X_I2CADDR, register);
    }

    private int readU16LE(int register) {
        return EndianReaders.readU16LE(this._rangeFinder, VL53L0X_I2CADDR, register);
    }

    private int readU16BE(int register){
        return EndianReaders.readU16BE(this._rangeFinder, VL53L0X_I2CADDR, register);
    }

    private int readS16LE(int register) {
        return EndianReaders.readS16LE(this._rangeFinder, VL53L0X_I2CADDR, register);
    }
    public int range()  {
        this._rangeFinder.write((byte)0x80, (byte)0x01);
        this._rangeFinder.write((byte)0xFF, (byte)0x01);
        this._rangeFinder.write((byte)0x00, (byte)0x00);
        this._rangeFinder.write((byte)0x91, (byte)this.stopVariable);
        this._rangeFinder.write((byte)0x00, (byte)0x01);
        this._rangeFinder.write((byte)0xFF, (byte)0x00);
        this._rangeFinder.write((byte)0x80, (byte)0x00);
        this._rangeFinder.write((byte)SYSRANGE_START, (byte)0x01);
        long start = System.currentTimeMillis();
        while ((this.readU8(SYSRANGE_START) & 0x01) > 0) {
            if (this.ioTimeousout > 0 && ((System.currentTimeMillis() - start) / 1_000) >= this.ioTimeout) {
                throw new RuntimeException("Timeout waiting for VL53L0X!");
            }
        }
        start = System.currentTimeMillis();
        while ((readU8(_rangeFinder, VL53L0X_I2CADDR, RESULT_INTERRUPT_STATUS,true) & 0x07) == 0) {
            if (this.ioTimeout > 0 && ((System.currentTimeMillis() - start) / 1_000) >= this.ioTimeout) {
                throw new RuntimeException("Timeout waiting for VL53L0X!");
            }
        }
        // assumptions: Linearity Corrective Gain is 1000 (default)
        // fractional ranging is not enabled
        int rangeMm = readU16BE(_rangeFinder, VL53L0X_I2CADDR,RESULT_RANGE_STATUS + 10, false);
        this._rangeFinder.write((byte)SYSTEM_INTERRUPT_CLEAR, (byte)0x01);
        return rangeMm;
    }

    @Override
    public void updateDashboard() {

    }
}
