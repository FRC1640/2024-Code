package frc.robot.sensors.I2CResolvers;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.I2C;
import lombok.Getter;

@Getter
public class I2CMuxResolver extends I2C {

    private I2C mux;
    final int m_channel;
    final int m_deviceAddress;
    private double v1;
    private double v2;
    private double angle1;
    private double angle2;
    private double offset;
    
    ArrayList<Double> average;

    public I2CMuxResolver(int channel, int deviceAddress, int muxPort, int muxAddress,
            double offset) {
        super(Port.values()[muxPort], deviceAddress);
        this.mux = new I2C(Port.values()[muxPort], muxAddress);
        m_channel = channel;
        m_deviceAddress = deviceAddress;
        this.offset = offset;
        
        average = new ArrayList<>(20);
    }

    // public String toString() {
    //     return String.format("%f, %f"); // TODO: ask justin
    // }

    

    public int getRawValue() {
        byte[] buffer = new byte[2];
        mux.writeBulk(new byte[] { (byte) (1 << m_channel) });
        this.read(0x03, 2, buffer);
        return (buffer[1] << 6);
    }

    // public long getLSBWeight(){
    // return ; //TODO: ask justin
    // }

    /**
     * @return Angle in radians
     */
    public double get () {
        return Math.toRadians(this.getD());
    }

    public double getD() {
        return (this.getRawValue() / 16384 * 360) + offset;
    }
}