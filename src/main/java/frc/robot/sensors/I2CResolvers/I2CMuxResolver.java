package frc.robot.sensors.I2CResolvers;

import edu.wpi.first.wpilibj.I2C;
import lombok.Getter;

@Getter
public class I2CMuxResolver extends I2C {

    private I2C mux;

    private final int m_deviceMuxChannel;
    private final int m_deviceAddress;
    private double offset;

    public I2CMuxResolver(I2C mux, int deviceAddress, int deviceMuxChannel, double offset) {
        super(Port.values()[mux.getPort()], deviceAddress);
        this.mux = mux;
        m_deviceMuxChannel = deviceMuxChannel;
        m_deviceAddress = deviceAddress;
        this.offset = offset;
    }

    /**
     * @return Raw encoder value
     */
    public int getRawValue() {
        mux.writeBulk(new byte[] { (byte) (1 << m_deviceMuxChannel) });
        byte[] buffer = new byte[2];
        this.read(0x03, 2, buffer);
        return ((Byte.toUnsignedInt(buffer[0]) << 6) | (Byte.toUnsignedInt(buffer[1]) >> 2));
    }

    /**
     * @return Angle in radians
     */
    public double getR() {
        return Math.toRadians(this.getD());
    }

    /**
     * @return Angle in degrees
     */
    public double getD() {
        return (this.getRawValue() / 16384 * 360) + offset;
    }

    public String toString() {
        return String.format("muxChannel:" + m_deviceMuxChannel + ", deviceAddress: " + m_deviceAddress);
    }

}