import lejos.nxt.*;

/**
 * This class is coming from the book Intelligence Unleashed by Brian Bagnall
 * isbn: 978-0-9868322-0-8
 */
public class Twirler implements ButtonListener {

    public static void main(String[] args) throws InterruptedException {
        Twirler twirler = new Twirler();
        Button.ESCAPE.addButtonListener(twirler);
        twirler.start();
    }

    public void start() throws InterruptedException {
        UltrasonicSensor ultrasonicSensor = new UltrasonicSensor(SensorPort.S1);

        Motor.B.setAcceleration(500);
        Motor.C.setAcceleration(500);
        Motor.B.forward();
        while(true) {
            if (ultrasonicSensor.getRange() < 40) {
                Sound.beep();
                Motor.B.setSpeed(800);
                Motor.C.setSpeed(800);
                Motor.B.backward();
                Motor.C.backward();
                Thread.sleep(4000);
                Motor.B.forward();
            }
            float speed1 = (float) Math.random() * 800;
            float speed2 = (float) Math.random() * 800;
            Motor.B.setSpeed(speed1);
            Motor.C.setSpeed(speed2);
            if (Math.random() < 0.5) {
                Motor.C.forward();
            } else {
                Motor.C.backward();
            }
            Thread.sleep(5000);
        }
    }

    public void buttonPressed(Button b) {
        // nothing for now
    }

    public void buttonReleased(Button b) {
        System.exit(0);
    }
}