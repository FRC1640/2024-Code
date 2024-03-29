/*----------------------------------------------------------------------------*/
/* Copyright (c) Kauai Labs 2015. All Rights Reserved.                        */
/*                                                                            */
/* Created in support of Team 2465 (Kauaibots).  Go Purple Wave!              */
/*                                                                            */
/* Open Source Software - may be modified and shared by FRC teams. Any        */
/* modifications to this code must be accompanied by the \License.txt file    */ 
/* in the root directory of the project.                                      */
/*----------------------------------------------------------------------------*/
package frc.robot.sensors.Gyro.Imported;

import com.kauailabs.navx.AHRSProtocol;

import edu.wpi.first.hal.SPIJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

class RegisterIO_SPI implements IRegisterIO{

    SPI port;
    int bitrate;
    boolean trace = false;
    int successive_error_count;

    static final int   DEFAULT_SPI_BITRATE_HZ         = 500000;
    static final int   NUM_IGNORED_SUCCESSIVE_ERRORS  = 50;
    
    public RegisterIO_SPI( SPI spi_port ) {
        port = spi_port;
        bitrate = DEFAULT_SPI_BITRATE_HZ;
        successive_error_count = 0;
    }
    
    public void enableLogging(boolean enable) {
    	trace = enable;
    }
    
    public RegisterIO_SPI( SPI spi_port, int bitrate ) {
        port = spi_port;
        this.bitrate = bitrate;
    }
    
    @Override
    public boolean init() {
        port.setClockRate(bitrate);
        port.setMode(SPI.Mode.kMode3);
        port.setChipSelectActiveLow();
        if (trace) Tracer.Trace("navX-MXP:  Initialized SPI communication at bitrate " + bitrate + "\n");
        return true;
    }

    @Override
    public boolean write(byte address, byte value ) {
        byte[] cmd = new byte[3];
        cmd[0] = (byte) (address  | (byte)0x80);
        cmd[1] = value;
        cmd[2] = AHRSProtocol.getCRC(cmd, 2);
        boolean write_ok;
        synchronized(this) {
        	write_ok = (port.write(cmd, cmd.length) == cmd.length);
        }
        if ( !write_ok ) {
            if (trace) Tracer.Trace("navX-MXP SPI Read:  Write error");
            return false; // WRITE ERROR
        }
        return true;
    }

    @Override
    public boolean read(byte first_address, byte[] buffer) {
        byte[] cmd = new byte[3];
        cmd[0] = first_address;
        cmd[1] = (byte)buffer.length;
        cmd[2] = AHRSProtocol.getCRC(cmd, 2);
        synchronized(this) {
	        if ( port.write(cmd, cmd.length) != cmd.length ) {
	        	return false; // WRITE ERROR
	        }
	        // delay 200 us /* TODO:  What is min. granularity of delay()? */
	        Timer.delay(0.001);
	        byte[] received_data = new byte[buffer.length+1];
            java.util.Arrays.fill(received_data, 0, buffer.length - 1, (byte)0x95 );
            received_data[buffer.length] = (byte)0x3E;
	        if ( port.read(true, received_data, received_data.length) != received_data.length ) {
                successive_error_count++;
                if (successive_error_count % NUM_IGNORED_SUCCESSIVE_ERRORS == 1) {
                    if (trace) {
                        System.out.printf("navX-MXP SPI Read:  Read error %s\n",
                        ((successive_error_count < NUM_IGNORED_SUCCESSIVE_ERRORS) ? "" : " (Repeated errors omitted)"));
                    }
                }                
	            return false; // READ ERROR
	        }
	        byte crc = AHRSProtocol.getCRC(received_data, received_data.length - 1);
	        if (( crc != received_data[received_data.length-1] ) ||
                ((received_data[0] == (byte)0x00) &&
                 (received_data[1] == (byte)0x00) &&
                 (received_data[2] == (byte)0x00) &&                    
                 (received_data[3] == (byte)0x00)))
            {
                successive_error_count++;
                if (successive_error_count % NUM_IGNORED_SUCCESSIVE_ERRORS == 1) {
                    if (trace) {
                        System.out.printf("navX-MXP SPI Read:  CRC error %s\n",
                        ((successive_error_count < NUM_IGNORED_SUCCESSIVE_ERRORS) ? "" : " (Repeated errors omitted)"));
                    }
                }
                resetSPI();
	            return false; // CRC ERROR
            }
            successive_error_count = 0;
	        System.arraycopy(received_data, 0, buffer, 0, received_data.length - 1);
        }
        return true;
    }

    private void resetSPI() {
    port.close();
    var portNumber = port.getPort();
    Timer.delay(0.001);
    SPIJNI.spiInitialize(portNumber);
    Timer.delay(0.010);
}

    @Override
    public boolean shutdown() {
        return true;
    }

}
