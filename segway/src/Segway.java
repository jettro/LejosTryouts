import lejos.nxt.Button;
import lejos.nxt.comm.RConsole;
import lejos.util.Delay;

/**
 * @author Jettro Coenradie
 * @author Anton Stevense
 */
public class Segway {
    public static void main(String[] args) {
        RConsole.open();

        Segway segway = new Segway();

        ReadGyroSensor gyroSensor = new ReadGyroSensor(segway);
        float gyroOffset = gyroSensor.calculateOffset();
        RConsole.println("Offset for gyro sensor : " + gyroOffset);
        gyroSensor.start();

        while(! Button.ESCAPE.isPressed()) {
            Delay.msDelay(100);
        }
        RConsole.close();
    }


}
