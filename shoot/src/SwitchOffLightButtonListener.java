import lejos.nxt.Button;
import lejos.nxt.ButtonListener;
import lejos.nxt.ColorSensor;

public class SwitchOffLightButtonListener implements ButtonListener {
    private final ColorSensor lightSensor;

    public SwitchOffLightButtonListener(ColorSensor lightSensor) {
        this.lightSensor = lightSensor;
    }

    public void buttonPressed(Button b) {
        lightSensor.setFloodlight(false);
    }

    public void buttonReleased(Button b) {
        //To change body of implemented methods use File | Settings | File Templates.
    }
}
