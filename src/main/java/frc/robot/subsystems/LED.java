package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{

    private static LED instance = null;
    private AddressableLED control = new AddressableLED(0);//TODO FILLER

    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(52);//TODO FILLER

    private final Color WHITE = new Color(200, 200, 200);
    private final Color GREEN = new Color(0, 200, 0);
    private final Color RED = new Color(255, 0, 0);
    private final Color CYAN = new Color(0,255,255);
    private final Color INIT_YELLOW = new Color(255,165,0);

    private LED(){
        super("LED");
        this.control.setLength(buffer.getLength());
        // this.control.setColorOrder(AddressableLED.ColorOrder.kRGB);
        this.setLights(INIT_YELLOW);
        this.control.setData(buffer);
        this.control.start();
    }

   
    public void setLights(Color color){
        for(int i = 0; i < buffer.getLength(); i++){
            buffer.setLED(i, color);
        }
    }

    public void setLights(Color col1, Color col2, Color col3){
        for(int i = 0; i < buffer.getLength()/2; i+=1){
            if(i % 3 == 0){
                buffer.setLED(2*i, col1);
                buffer.setLED(2*i+1, col1);
            }
            else if(i % 3 == 1){
                buffer.setLED(2*i, col2);
                buffer.setLED(2*i+1, col2);
            }
            else{
                buffer.setLED(2*i, col3);
                buffer.setLED(2*i+1, col3);
            };
        }
    }

    @Override
    public void periodic(){
        if(Intake.getInstance().isIntaking()) this.setLights(CYAN);
        else if(Shooter.getInstance().isShooting()) this.setLights(RED);
        else this.setLights(RED, GREEN, WHITE);

        this.control.setData(buffer);
    }

    public static LED getInstance(){
        if(LED.instance == null)
            LED.instance = new LED();
        return LED.instance;
    }

}