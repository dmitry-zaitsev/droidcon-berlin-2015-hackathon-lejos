package org.lejos.example;

import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.RangeReading;
import lejos.robotics.objectdetection.*;

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

        while (System.currentTimeMillis() - startTime < interval) {
            Feature feature = detector.scan();
            if (feature == null) {
                continue;
            }

            RangeReading reading = feature.getRangeReading();
            System.out.println(reading.getAngle() + " " + reading.getRange());
        }
    }

    private static RangeFeatureDetector ultrasonicDetectorForPort(SensorPort port) {
        return new RangeFeatureDetector(new UltrasonicSensor(port), 40f, 250);
    }
}
