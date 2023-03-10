package frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import java.util.ArrayList;


public class LED extends SubsystemBase{
    ArrayList<Byte> byteList = new ArrayList<Byte>();
    private static final int kDeviceAddress = 4;
    private I2C i2c;
    byte data = 0;
    byte oldData = 0;
    int i = 0;
    int j = 0;

    public LED() {
        i2c = new I2C(I2C.Port.kMXP, kDeviceAddress);
    }

    private void writeByte(int input) {

        byte data = (byte) input;
    
        // Writes bytes over I2C
        i2c.write(0, data);
    }

    public void clearLED() {
        byteList.add((byte)0);
    }

    public void setLED(int strip, int function) {
        data = (byte)((strip*16)|function);
        byteList.add(data);
    }

    @Override
    public void periodic(){
        if (j == 0){
            setLED(1, 4);
            setLED(2, 4);
        }
        if (i == 10 ) {
            if (byteList.size() > 0)
            {
            writeByte(byteList.get(0));
            byteList.remove(0);
            }
            i = 0;

        }
        ++j;
        ++i;
    }
   
}