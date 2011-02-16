import lejos.nxt.LCD;
import lejos.nxt.SensorPort;
import lejos.nxt.TouchSensor;
import lejos.nxt.addon.GyroSensor;
import lejos.nxt.addon.TiltSensor;
import lejos.util.Delay;

public class SensorTest {
    private final TouchSensor touchSensor;
    private final GyroSensor gyroSensor;
    private final TiltSensor tiltSensor;

    public static void main(String[] args) {
        SensorTest sensorTest = new SensorTest();

//        sensorTest.readTouchSensor();
//        sensorTest.readGyroSensor();
        sensorTest.readAccelSensor();
    }

    private void readAccelSensor() {
        int x, y, z;
        do {
            Delay.msDelay(10);
            x = tiltSensor.getXTilt();
            y = tiltSensor.getYTilt();
            z = tiltSensor.getZTilt();
            LCD.drawString("x : " + x, 0, 1);
            LCD.drawString("y : " + y, 0, 2);
            LCD.drawString("z : " + z, 0, 3);
        } while (!touchSensor.isPressed());
    }

    private void readGyroSensor() {
        int i;
        do {
            Delay.msDelay(10);
            i = gyroSensor.readValue();
            LCD.drawString("output : " + i, 0, 1);

        } while (!touchSensor.isPressed());
    }

    private void readTouchSensor() {
        long i = 0;
        do {
            Delay.msDelay(10);
            i += 10;

            LCD.drawString("ms : " + i, 0, 1);

        } while (!touchSensor.isPressed());

    }

    public SensorTest() {
        touchSensor = new TouchSensor(SensorPort.S2);
        gyroSensor = new GyroSensor(SensorPort.S1);
        tiltSensor = new TiltSensor(SensorPort.S1);
    }
}
