package frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.ArrayList;

public class LED extends SubsystemBase{
    final public static int PURPLE = 1;
    final public static int YELLOW = 2;
    final public static int RED = 3;
    final public static int RAINBOW = 4;
    final public static int WHITE = 5;

    final public static int EPANEL = 1; //Can support up to 7 strips, names should be re-mapped at some point
    final public static int TRIGHT = 2;
    final public static int TLEFT = 3;
    final public static int PPANEL = 4;
    final public static int ALL = 15;

    private final int ALLIANCE_COLOR_BLUE = 6;
    private final int ALLIANCE_COLOR_RED = 7;

    int led_color_state;

    public static int m_alliance_color;

    public ArrayList<Integer> dataList = new ArrayList<Integer>();
    private static final int kDeviceAddress = 4;
    private I2C i2c;
    private int i = 0;

    public LED() {
        i2c = new I2C(I2C.Port.kMXP, kDeviceAddress);
    }

    private void writeData(int input) {
        i2c.write(0, input);
    }

    public void clearLED() {
        dataList.add(0);
    }

    public void setLED(int strip, int function) {
        int data = (strip*16)|function;
        dataList.add(data);
    }
    
    public CommandBase ResendLEDBytes() {
		return runOnce(
				() -> {
                setLED(LED.ALL, LED.RAINBOW);
				});
	}

    @Override
    public void periodic() {
        if (dataList.size() > 0 && i >= 10) {
            writeData(dataList.get(0));
            dataList.remove(0);
            i = 0;
        }
        ++i;

        if( DriverStation.getAlliance() == Alliance.Blue){
            m_alliance_color = ALLIANCE_COLOR_BLUE;
          }
          else {
            m_alliance_color = ALLIANCE_COLOR_RED;
          }


    switch(led_color_state){
        default:
          if(m_alliance_color == ALLIANCE_COLOR_BLUE){
          }
          else{
          }
          break;
      }
    }


}