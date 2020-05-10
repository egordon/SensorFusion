package com.ethankgordon.sensorfusion;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.core.content.res.TypedArrayUtils;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.provider.ContactsContract;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import com.jjoe64.graphview.GraphView;
import com.jjoe64.graphview.series.DataPoint;
import com.jjoe64.graphview.series.DataPointInterface;
import com.jjoe64.graphview.series.PointsGraphSeries;

import org.apache.commons.math3.complex.Quaternion;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.stat.descriptive.moment.Mean;
import org.apache.commons.math3.stat.descriptive.moment.StandardDeviation;

import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Vector;

@SuppressWarnings("unchecked")
public class MainActivity extends AppCompatActivity implements SensorEventListener, View.OnClickListener {
    // Sensors
    private SensorManager sensorManager;
    private Sensor mGyro;
    private Sensor mAccel;

    // Calibration Variables
    private final int NUM_CALIBRATION = 500;
    private boolean isGyroCalibrating = true;
    private boolean isAccelCalibrating = true;
    private int numGyroCalib = 0;
    private int numAccelCalib = 0;
    private List<List<Double>> accelCalcs;
    private List<List<Double>> gyroCalcs;

    // Math Variables
    private Vector3D gyroBias = Vector3D.ZERO;
    private Rotation gyroOnlyRot = Rotation.IDENTITY;
    private Rotation compRot = Rotation.IDENTITY;

    //complementary filter value [0,1].
    // 1: ignore acc tilt, 0: use all acc tilt
    private final double ALPHA = 0.9;


    // Graph Variables
    private GraphView[] graphs;
    private PointsGraphSeries[] realTimeSeries;
    private PointsGraphSeries[] traceSeries;
    private ArrayList<DataPoint>[] traceData;


    // Visual Variables
    private Button calibButton;
    private Button saveButton;
    private TextView[] accelBiasText;
    private TextView[] accelNoiseText;
    private TextView[] gyroBiasText;
    private TextView[] gyroNoiseText;

    private DecimalFormat formatter = new DecimalFormat("#.0#####", DecimalFormatSymbols.getInstance( Locale.ENGLISH ));

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Init Sensors
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        mGyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mAccel = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

        // Init Calibration Elements
        gyroCalcs = new ArrayList<List<Double>>(3);
        for(int i = 0; i < 3; i++) {
            gyroCalcs.add(new ArrayList<Double>(NUM_CALIBRATION));
        }
        accelCalcs = new ArrayList<List<Double>>(3);
        for(int i = 0; i < 3; i++) {
            accelCalcs.add(new ArrayList<Double>(NUM_CALIBRATION));
        }

        // Init Graphs
        graphs = new GraphView[3];
        graphs[0] = findViewById(R.id.gyroOnly);
        graphs[0].setTitle("Gyro Only");
        graphs[1] = findViewById(R.id.accelOnly);
        graphs[1].setTitle("Accel Only");
        graphs[2] = findViewById(R.id.complementary);
        graphs[2].setTitle("Complementary");
        realTimeSeries = new PointsGraphSeries[3];
        traceSeries = new PointsGraphSeries[3];
        traceData = new ArrayList[3];

        for(int i = 0; i < 3; i++) {
            // Set Graph Params
            graphs[i].getViewport().setXAxisBoundsManual(true);
            graphs[i].getViewport().setMinX(-0.5);
            graphs[i].getViewport().setMaxX(0.5);
            graphs[i].getViewport().setYAxisBoundsManual(true);
            graphs[i].getViewport().setMinY(-0.5);
            graphs[i].getViewport().setMaxY(0.5);

            // Set Series Params
            realTimeSeries[i] = new PointsGraphSeries<DataPoint>();
            realTimeSeries[i].appendData(new DataPoint(0.0, 0.0), false, 1);
            realTimeSeries[i].setColor(Color.RED);
            realTimeSeries[i].setCustomShape(new PointsGraphSeries.CustomShape() {
                @Override
                public void draw(Canvas canvas, Paint paint, float x, float y, DataPointInterface dataPoint) {
                    paint.setStrokeWidth(10);
                    canvas.drawLine(x-20, y-20, x+20, y+20, paint);
                    canvas.drawLine(x+20, y-20, x-20, y+20, paint);
                }
            });


            traceSeries[i] = new PointsGraphSeries<DataPoint>();
            traceSeries[i].setColor(Color.BLACK);
            traceSeries[i].setShape(PointsGraphSeries.Shape.POINT);
            traceSeries[i].setSize(10.0f);
            graphs[i].addSeries(traceSeries[i]);
            graphs[i].addSeries(realTimeSeries[i]);

            traceData[i] = new ArrayList<DataPoint>();
        }

        // Init Visual Elements
        calibButton = findViewById(R.id.recalibrate);
        calibButton.setOnClickListener(this);
        saveButton = findViewById(R.id.savePlots);
        saveButton.setOnClickListener(this);

        gyroBiasText = new TextView[3];
        gyroBiasText[0] = findViewById(R.id.gyroBiasX);
        gyroBiasText[1] = findViewById(R.id.gyroBiasY);
        gyroBiasText[2] = findViewById(R.id.gyroBiasZ);
        gyroNoiseText = new TextView[3];
        gyroNoiseText[0] = findViewById(R.id.gyroNoiseX);
        gyroNoiseText[1] = findViewById(R.id.gyroNoiseY);
        gyroNoiseText[2] = findViewById(R.id.gyroNoiseZ);

        accelBiasText = new TextView[3];
        accelBiasText[0] = findViewById(R.id.accelBiasX);
        accelBiasText[1] = findViewById(R.id.accelBiasY);
        accelBiasText[2] = findViewById(R.id.accelBiasZ);
        accelNoiseText = new TextView[3];
        accelNoiseText[0] = findViewById(R.id.accelNoiseX);
        accelNoiseText[1] = findViewById(R.id.accelNoiseY);
        accelNoiseText[2] = findViewById(R.id.accelNoiseZ);

    }

    @Override
    protected void onStart() {
        super.onStart();
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE)
                != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE},0);
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        // Register sensor listeners
        sensorManager.registerListener(this,
                mAccel,
                SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this,
                mGyro,
                SensorManager.SENSOR_DELAY_FASTEST);
    }

    @Override
    protected void onPause() {
        // unregister listeners
        super.onPause();
        sensorManager.unregisterListener(this);
    }

    private long prevTimestamp = 0L;
    private double[] accelSave = {0.0, 0.0, 0.0};
    @Override
    public void onSensorChanged(SensorEvent event) {
        Sensor sensor = event.sensor;
        if (sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            accelSave = convertFloatsToDoubles(event.values);
            handleAccel(accelSave);
        } else if (sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            if (prevTimestamp == 0L) {
                prevTimestamp = event.timestamp;
                return;
            }
            double deltaT = 1E-9 * (event.timestamp - prevTimestamp);
            prevTimestamp = event.timestamp;
            handleGyro(convertFloatsToDoubles(event.values), deltaT);

            // Do Complementary Filter
            if (!isAccelCalibrating && !isGyroCalibrating) {
                // Turn on calibration button
                if (!calibButton.isEnabled()) {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            calibButton.setText("Calibrate");
                            calibButton.setEnabled(true);
                        }
                    });
                }

                // Run Complementary Filter
                // Calculate Tilt from Gyro
                Vector3D gyroVec = new Vector3D(convertFloatsToDoubles(event.values));
                gyroVec = gyroVec.subtract(gyroBias);

                Rotation qDelta = Rotation.IDENTITY;

                if(gyroVec.getNorm() > 1E-8) {
                    qDelta = new Rotation(gyroVec.normalize(), deltaT * gyroVec.getNorm(), RotationConvention.VECTOR_OPERATOR);
                }

                Rotation qw = qDelta.applyTo(compRot);


                // Get Accelerometer in world
                Vector3D accel = new Vector3D(accelSave);
                accel = qw.applyTo(accel);

                // Compute Tilt Correction
                double phi = Math.acos(accel.getZ() / accel.getNorm());
                double normN = Math.sqrt(accel.getX()*accel.getX() + accel.getY()*accel.getY());
                Rotation qt = Rotation.IDENTITY;
                if(normN > 1E-8) {
                    Vector3D axis = new Vector3D(accel.getY()/normN, -accel.getX()/normN, 0.0);
                    qt = new Rotation(axis, (1.0-ALPHA)*phi, RotationConvention.VECTOR_OPERATOR);
                }

                compRot = qt.applyTo(qw);

                // Draw
                final Vector3D drawVec = compRot.applyTo(Vector3D.PLUS_K);
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        addData(2, drawVec.getX(), drawVec.getY());
                    }
                });

            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Pass
    }

    private static double[] convertFloatsToDoubles(float[] input)
    {
        if (input == null)
        {
            return null; // Or throw an exception - your choice
        }
        double[] output = new double[input.length];
        for (int i = 0; i < input.length; i++)
        {
            output[i] = input[i];
        }
        return output;
    }

    private static double[] unboxDoubles(List<Double> input)
    {
        if (input == null)
        {
            return null; // Or throw an exception - your choice
        }
        double[] output = new double[input.size()];
        for (int i = 0; i < input.size(); i++)
        {
            output[i] = input.get(i);
        }
        return output;
    }


    private void handleAccel(double[] accel) {
        if (isAccelCalibrating) {
            for(int i = 0; i < 3; i++) {
                accelCalcs.get(i).add(numAccelCalib, accel[i]);
            }
            numAccelCalib++;
            if(numAccelCalib >= NUM_CALIBRATION) {
                final double[] accelMean = {0.0, 0.0, 0.0};
                double[] accelSD = {0.0, 0.0, 0.0};

                Mean meanCalc = new Mean();
                StandardDeviation sdCalc = new StandardDeviation();
                for(int i = 0; i < 3; i++) {
                    // Calculate Mean
                    accelMean[i] = meanCalc.evaluate(unboxDoubles(accelCalcs.get(i)));
                    accelSD[i] = sdCalc.evaluate(unboxDoubles(accelCalcs.get(i)));
                }

                // Write Mean and SD to UI
                final double[] accelBiasUI = accelMean;
                final double[] accelNoiseUI = accelSD;
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        for(int i = 0; i < 3; i++) {
                            accelBiasText[i].setText(formatter.format(accelBiasUI[i]));
                            accelNoiseText[i].setText(formatter.format(accelNoiseUI[i]));
                        }
                    }
                });
                isAccelCalibrating = false;
            }
            return;
        }

        // Simple Tilt from Accelerometer
        Vector3D accelVec = new Vector3D(accel);

        // Draw
        final Vector3D drawVec = Vector3D.ZERO.subtract(accelVec.normalize());
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                addData(1, drawVec.getX(), drawVec.getY());
            }
        });


    }

    private void handleGyro(double[] gyro, double deltaT) {
        if (isGyroCalibrating) {
            for(int i = 0; i < 3; i++) {
                gyroCalcs.get(i).add(numGyroCalib, gyro[i]);
            }
            numGyroCalib++;
            if(numGyroCalib >= NUM_CALIBRATION) {
                final double[] gyroMean = {0.0, 0.0, 0.0};
                double[] gyroSD = {0.0, 0.0, 0.0};

                Mean meanCalc = new Mean();
                StandardDeviation sdCalc = new StandardDeviation();
                for(int i = 0; i < 3; i++) {
                    // Calculate Mean
                    gyroMean[i] = meanCalc.evaluate(unboxDoubles(gyroCalcs.get(i)));
                    gyroSD[i] = sdCalc.evaluate(unboxDoubles(gyroCalcs.get(i)));
                }

                // Write Mean and SD to UI
                final double[] gyroBiasUI = gyroMean;
                final double[] gyroNoiseUI = gyroSD;
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        for(int i = 0; i < 3; i++) {
                            gyroBiasText[i].setText(formatter.format(gyroBiasUI[i]));
                            gyroNoiseText[i].setText(formatter.format(gyroNoiseUI[i]));
                        }
                    }
                });
                gyroBias = new Vector3D(gyroMean);
                isGyroCalibrating = false;
            }
            return;
        }

        // Calculate Tilt from Gyro
        Vector3D gyroVec = new Vector3D(gyro);
        // It's already calibrated, according to Android Docs.
        //gyroVec = gyroVec.subtract(gyroBias);

        Rotation qDelta = Rotation.IDENTITY;

        if(gyroVec.getNorm() > 1E-8) {
            qDelta = new Rotation(gyroVec.normalize(), deltaT * gyroVec.getNorm(), RotationConvention.VECTOR_OPERATOR);
        }

        gyroOnlyRot = qDelta.applyTo(gyroOnlyRot);

        // Draw
        final Vector3D drawVec = gyroOnlyRot.applyTo(Vector3D.PLUS_K);
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                addData(0, drawVec.getX(), drawVec.getY());
            }
        });
    }


    @Override
    public void onClick(View v) {
        if(v.getId() == calibButton.getId()) {
            // Start Calibration Procedure
            for (int i = 0; i < 3; i++) {
                gyroBiasText[i].setText("");
                gyroNoiseText[i].setText("");
                accelBiasText[i].setText("");
                accelNoiseText[i].setText("");

                traceData[i].clear();
            }
            numAccelCalib = 0;
            numGyroCalib = 0;
            calibButton.setEnabled(false);
            calibButton.setText("Calibrating...");

            // Reset Quaterions
            gyroOnlyRot = Rotation.IDENTITY;
            compRot = Rotation.IDENTITY;
            isAccelCalibrating = true;
            isGyroCalibrating = true;
        } else if(v.getId() == saveButton.getId()) {
            graphs[0].takeSnapshotAndShare(this, "gyro_only", "GyroOnlyPlot");
            graphs[1].takeSnapshotAndShare(this, "accel_only", "AccelOnlyPlot");
            graphs[2].takeSnapshotAndShare(this, "complementary", "ComplementaryFilter");
        }
    }

    private int[] traceCounter = {0, 0, 0};
    private void addData(int index, double x, double y) {
        DataPoint value = new DataPoint(x, y);
        realTimeSeries[index].resetData(new DataPoint[] { value });

        if(traceCounter[index] > 100) {
            traceData[index].add(value);
            Collections.sort(traceData[index], new Comparator<DataPoint>() {
                @Override
                public int compare(DataPoint o1, DataPoint o2) {
                    if (o1.getX() == o2.getX()) {
                        return 0;
                    }
                    return (o1.getX() > o2.getX()) ? 1 : -1;
                }
            });
            traceSeries[index].resetData(traceData[index].toArray(new DataPoint[0]));
            traceCounter[index] = 0;
        }
        traceCounter[index]++;
    }
}
