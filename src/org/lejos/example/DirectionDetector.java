package org.lejos.example;

import lejos.robotics.RangeReading;
import lejos.robotics.RangeReadings;
import lejos.robotics.objectdetection.Feature;
import lejos.robotics.objectdetection.FeatureDetectorAdapter;
import lejos.robotics.objectdetection.RangeFeatureDetector;

/**
 * Detects an angle to the object
 */
public class DirectionDetector extends FeatureDetectorAdapter {

    private final RangeFeatureDetector leftDetector;
    private final RangeFeatureDetector rightDetector;
    private final float distanceBetweenSensors;

    public DirectionDetector(RangeFeatureDetector leftDetector,
                             RangeFeatureDetector rightDetector,
                             float distanceBetweenSensors) {
        super(
                Math.max(
                        leftDetector.getDelay(),
                        rightDetector.getDelay()
                )
        );

        this.leftDetector = leftDetector;
        this.rightDetector = rightDetector;
        this.distanceBetweenSensors = distanceBetweenSensors;
    }

    @Override
    public Feature scan() {
        Feature leftFeature = leftDetector.scan();
        Feature rightFeature = rightDetector.scan();

        if (leftFeature == null && rightFeature == null) {
            return null;
        }

        return calculateFeature(leftFeature, rightFeature);
    }

    private Feature calculateFeature(Feature leftFeature, Feature rightFeature) {
        final float leftRange = leftFeature != null
                ? leftFeature.getRangeReading().getRange()
                : Float.POSITIVE_INFINITY;

        final float rightRange = rightFeature != null
                ? rightFeature.getRangeReading().getRange()
                : Float.POSITIVE_INFINITY;

        final float range = calculateRange(leftFeature, rightFeature);

        float angle = calculateAngle(leftFeature, rightFeature, range);
        if (Float.isNaN(angle)) {
            if (leftRange < rightRange) {
                angle = -45f;
            } else if (leftRange > rightRange) {
                angle = 45f;
            } else {
                angle = 0f;
            }
        }

        return new DirectionFeature(
                angle,
                range
        );
    }

    private float calculateAngle(Feature leftFeature, Feature rightFeature, float range) {
        if (leftFeature == null) {
            return -45f;
        } else if (rightFeature == null) {
            return 45f;
        }

        final float hd = distanceBetweenSensors / 2f;
        final float sensorRange = leftFeature.getRangeReading().getRange();

        final float cosValue = (range * range + hd * hd - sensorRange * sensorRange) / (2 * range * hd);

        return (float) Math.toDegrees(
                Math.acos(cosValue)
        ) - 90f;
    }

    private float calculateRange(Feature leftFeature, Feature rightFeature) {
        final float leftRange = leftFeature != null
                ? leftFeature.getRangeReading().getRange()
                : Float.POSITIVE_INFINITY;

        final float rightRange = rightFeature != null
                ? rightFeature.getRangeReading().getRange()
                : Float.POSITIVE_INFINITY;

        if (leftFeature != null && rightFeature != null) {
            final float c = rightFeature.getRangeReading().getRange();
            final float b = leftFeature.getRangeReading().getRange();
            final float a = distanceBetweenSensors;

            return (float) (0.5 * Math.sqrt(2 * c * c + 2 * b * b - a * a));
        } else {
            return Math.min(leftRange, rightRange);
        }
    }

    private static class DirectionFeature implements Feature {

        private final RangeReading reading;
        private final long timestamp;

        DirectionFeature(float angle, float range) {
            reading = new RangeReading(angle, range);
            timestamp = System.currentTimeMillis();
        }

        @Override
        public RangeReading getRangeReading() {
            return reading;
        }

        @Override
        public RangeReadings getRangeReadings() {
            throw new UnsupportedOperationException();
        }

        @Override
        public long getTimeStamp() {
            return timestamp;
        }
    }

}
