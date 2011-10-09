import lejos.nxt.Button;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;

/**
 * This class is coming from the book Intelligence Unleashed by Brian Bagnall
 * isbn: 978-0-9868322-0-8
 */
public class DiffCar {
    public static void main(String[] args) {
        double diam = DifferentialPilot.WHEEL_SIZE_NXT2;
        double trackWidth = 15.5;

        DifferentialPilot pilot = new DifferentialPilot(diam,trackWidth, Motor.B,Motor.C);
        UltrasonicSensor ultrasonicSensor = new UltrasonicSensor(SensorPort.S1);
        pilot.forward();
        while(!Button.ESCAPE.isPressed()) {
            if (ultrasonicSensor.getDistance() < 40) {
                pilot.travel(-20);
                pilot.rotate(45);
                pilot.forward();
            }
        }
    }
}
