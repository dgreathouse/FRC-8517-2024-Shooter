

package frc.robot.lib;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDs {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    /** Number of LEDs in the strip. Change this number to match the number of LEDs */
    private int m_numLEDs = 5;
    private int m_red = 0;
    private int m_green = 0;
    private int m_blue = 0;
    

    /** This is a constructor for the class. It has the same name as the class and has no return type.
     * 
     * @param _port The PWM port the LEDs are plugged into
     */
    public LEDs(int _port){
        m_led = new AddressableLED(_port);
        m_ledBuffer = new AddressableLEDBuffer(m_numLEDs);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    /** Set all the LEDs to the requested RGB values
     * 
     * @param _r Red Value between 0-255
     * @param _g Green Value between 0-255
     * @param _b Blue Value between 0-255
     */
    public void setRGBColor(int _r, int _g, int _b){
        if(_r == m_red && _g == m_green && _b == m_blue){
            // do nothing since the colors are already set to what was requested
        }else {
            m_red = _r; m_green = _g; m_blue = _b;
            for(int i = 0; i < m_ledBuffer.getLength(); i++){
                m_ledBuffer.setRGB(i, m_red, m_green, m_blue);
            }
            m_led.setData(m_ledBuffer);
        }
        
    }
    /** Set all the LEDs to the Color8Bit value
     *  
     * @param _color The color to set
     */
    public void setRGBColor(Color8Bit _color){
        setRGBColor(_color.red, _color.green, _color.blue);
    }
    /**
     * Set the LEDs to the color of the alliance the team is on.
     * If the the robot is not connected to the Field Management System (FMS)
     * then the color is green.
     */
    public void setAllianceColor(){
       Color8Bit color = new Color8Bit();
        if(DriverStation.getAlliance().get() == Alliance.Blue){
            color = new Color8Bit(0, 0, 200);
        }else if(DriverStation.getAlliance().get() == Alliance.Red){
            color = new Color8Bit(200, 0, 0);
        }else {
            color = new Color8Bit(0, 200, 0);
        }
        setRGBColor(color);
    }
    /** Set the color to green if difference between requested and actual are within the tolerance
     * 
     * @param _requested The value that is requested
     * @param _actual The actual value
     * @param _tolerance The tolerance
     */
    public void setPositionAccuaracy(double _requested, double _actual, double _tolerance){
        double error = _requested - _actual;
        if(Math.abs(error) < _tolerance){
            setRGBColor(0, 200, 0);
        }else {
            setAllianceColor();
        }
    }
}
