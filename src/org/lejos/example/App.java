package org.lejos.example;

import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.RangeReading;
import lejos.robotics.objectdetection.*;
import lejos.nxt.Motor;

/**
 * Example leJOS Project with an ant build file
 */
public class App {

    private static final float DISTANCE_BETWEEN_SENSORS_CM = 5f;

    public static void main(String[] args) throws Exception {
        final long interval = 60000;
        final long startTime = System.currentTimeMillis();

        DirectionDetector detector = new DirectionDetector(
                ultrasonicDetectorForPort(SensorPort.S1),
                ultrasonicDetectorForPort(SensorPort.S4),
                DISTANCE_BETWEEN_SENSORS_CM
        );
        final HackPilot p = new HackPilot(5f, Motor.A, Motor.B, 41, -150, 150);

        while (System.currentTimeMillis() - startTime < interval) {
            Feature feature = detector.scan();
            if (feature == null) {
            	p.travel(10);
                continue;
            }

            RangeReading reading = feature.getRangeReading();
            System.out.println(reading.getAngle() + " " + reading.getRange());
            if (reading.getAngle() > 0) {
            	p.travelArc(600, 10);
            } else {
            	p.travelArc(-600, 10);            	
            }
        }
    }

    private static RangeFeatureDetector ultrasonicDetectorForPort(SensorPort port) {
        return new RangeFeatureDetector(new UltrasonicSensor(port), 40f, 250);
    }
}
