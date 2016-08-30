package intelligence.dummy.dip;

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.support.v4.app.ActivityCompat;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.RadioGroup;
import android.widget.Switch;
import android.widget.TextView;

import com.androidplot.xy.BoundaryMode;
import com.androidplot.xy.LineAndPointFormatter;
import com.androidplot.xy.SimpleXYSeries;
import com.androidplot.xy.XYPlot;
import com.androidplot.xy.XYSeries;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.Timer;
import java.util.TimerTask;

public class SensorFusionActivity extends Activity
        implements SensorEventListener, RadioGroup.OnCheckedChangeListener, Switch.OnCheckedChangeListener, Button.OnClickListener {

    private SensorManager mSensorManager = null;
    // угловая скорость с гироскопа
    private float[] gyro = new float[3];
    // матрица поворота по данным с гироскопа
    private float[] gyroMatrix = new float[9];
    // углы ориентации по данным гироскопа
    private float[] gyroOrientation = new float[3];
    // вектор магнитного поля
    private float[] magnet = new float[3];
    // вектор акселерометра
    private float[] accel = new float[3];
    // углы ориентации по данным акселерометра и магнетометра
    private float[] accMagOrientation = new float[3];
    // окончательные углы ориентации, полученные с помощью sensor fusion
    private float[] fusedOrientation = new float[3];
    // матрица поворота, построенная по данным акселерометра и магнетометра
    private float[] rotationMatrix = new float[9];

    public static final float EPSILON = 0.000000001f;
    private static final float NS2S = 1.0f / 1000000000.0f;
    private long timestamp;
    private boolean initState = true;

    public static final int TIME_CONSTANT = 30;
    public static final float FILTER_COEFFICIENT = 0.98f;
    private Timer fuseTimer = new Timer();

    // Эти элементы нужны для визуализации
    public Handler mHandler;
    private Switch sw;
    private int radioSelection1, radioSelection2;

    private XYPlot plot;
    private static final int len = 50;
    private Number[] seriesAzimuth = new Number[len];
    private Number[] seriesPitch = new Number[len];
    private Number[] seriesRoll = new Number[len];
    private LineAndPointFormatter series1Format;
    private XYSeries series1;
    private DrawView mDrawView;
    private long oldTime;
    private double distance;
    private double azimuth;
    private double ac;
    private final int REQUEST_EXTERNAL_STORAGE = 1;
    private String[] PERMISSIONS_STORAGE = {
            Manifest.permission.READ_EXTERNAL_STORAGE,
            Manifest.permission.WRITE_EXTERNAL_STORAGE
    };
    private FileOutputStream stream;
    private boolean granted;

    int iter;
    private static final int MAX_ITER = 1000;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        granted = false;
        iter = 0;

        oldTime = 0;
        distance = 0;

        gyroOrientation[0] = 0.0f;
        gyroOrientation[1] = 0.0f;
        gyroOrientation[2] = 0.0f;

        // инициализируем gyroMatrix единичной матрицей
        gyroMatrix[0] = 1.0f; gyroMatrix[1] = 0.0f; gyroMatrix[2] = 0.0f;
        gyroMatrix[3] = 0.0f; gyroMatrix[4] = 1.0f; gyroMatrix[5] = 0.0f;
        gyroMatrix[6] = 0.0f; gyroMatrix[7] = 0.0f; gyroMatrix[8] = 1.0f;

        // получаем менеджер сенсоров и инициализируем отслеживание датчиков
        mSensorManager = (SensorManager) this.getSystemService(SENSOR_SERVICE);
        initListeners();

        // ждем секунду, пока данные гироскопа и магнетометра/акселерометра
        // не проинициализируются, и назначаем задачу комплементарного фильтра
        fuseTimer.scheduleAtFixedRate(new calculateFusedOrientationTask(),
                1000, TIME_CONSTANT);

        // элементы пользовательского интерфейса
        mHandler = new Handler();
        radioSelection1 = 0;
        radioSelection2 = 0;
        RadioGroup mRadioGroup1 = (RadioGroup) findViewById(R.id.radioGroup1);
        mRadioGroup1.setOnCheckedChangeListener(this);
        RadioGroup mRadioGroup2 = (RadioGroup) findViewById(R.id.radioGroup2);
        mRadioGroup2.setOnCheckedChangeListener(this);
        sw = (Switch)findViewById(R.id.switch1);
        sw.setOnCheckedChangeListener(this);
        mDrawView = (DrawView)findViewById(R.id.draw);

        // элементы графиков
        plot = (XYPlot) findViewById(R.id.plot);
        plot.setRangeBoundaries(-180, 180, BoundaryMode.FIXED);

        Arrays.fill(seriesAzimuth,0);
        Arrays.fill(seriesRoll,0);
        Arrays.fill(seriesPitch,0);

        // превращаем массив в XYSeries':
        // (Y_VALS_ONLY значит, что мы используем индекс, как значение x)
        series1 = new SimpleXYSeries(Arrays.asList(seriesAzimuth),
                SimpleXYSeries.ArrayFormat.Y_VALS_ONLY, "Series1");

        // конфигурируем форматирование
        series1Format = new LineAndPointFormatter();
        series1Format.setPointLabelFormatter(null);
        series1Format.configure(getApplicationContext(),
                R.xml.line_point_formatter);

        // добавляем массив на график
        plot.addSeries(series1, series1Format);

        // сокращаем количество меток диапазона
        plot.setTicksPerRangeLabel(3);

        // поворачиваем метки на 45 градусов для компактности:
        plot.getGraphWidget().setDomainLabelOrientation(-45);
    }

    @Override
    public void onStop() {
        super.onStop();
        // отключаем отслеживание датчиков чтобы предотвратить ненужный расход энергии
        mSensorManager.unregisterListener(this);
    }

    @Override
    protected void onPause() {
        super.onPause();
        // отключаем отслеживание датчиков чтобы предотвратить ненужный расход энергии
        mSensorManager.unregisterListener(this);
    }

    @Override
    public void onResume() {
        super.onResume();
        // восстанавливаем отслеживание датчиков, когда пользователь возвращает приложение
        initListeners();
    }

    // функция регистрирует отслеживание датчиков для акселерометра, магнетометра и гироскопа
    public void initListeners(){
        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_FASTEST);

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED),
                SensorManager.SENSOR_DELAY_FASTEST);

        mSensorManager.registerListener(this,
                mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_FASTEST);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        switch(event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                // копируем новые данные акселерометра в массив accel и вычисляем ориентацию
                System.arraycopy(event.values, 0, accel, 0, 3);
                calculateAccMagOrientation();
                long newTime = System.currentTimeMillis();
                long dTime;
                if(oldTime == 0) {
                    dTime = 0;
                }
                else {
                    dTime = newTime - oldTime;
                }
                oldTime = newTime;
                calculateDistance(dTime);
                break;

            case Sensor.TYPE_GYROSCOPE_UNCALIBRATED:
                // обрабатываем данные гироскопа
                gyroFunction(event);
                break;

            case Sensor.TYPE_MAGNETIC_FIELD:
                // копируем новые данные магнетометра в массив magnet
                System.arraycopy(event.values, 0, magnet, 0, 3);
                break;
        }
    }

    // вычисляем углы ориентации по данным акселерометра и магнетометра
    public void calculateAccMagOrientation() {
        if(SensorManager.getRotationMatrix(rotationMatrix, null, accel, magnet)) {
            SensorManager.getOrientation(rotationMatrix, accMagOrientation);
        }
    }

    public void calculateDistance(long time) {
        final float alpha = 0.8f;
        float[] gravity = {0,0,0};
        float[] linAc ={0,0,0};

        // ФВЧ. Отсечение влияния гравитации
        /*gravity[0] = alpha * gravity[0] + (1 - alpha) * accel[0];
        gravity[1] = alpha * gravity[1] + (1 - alpha) * accel[1];
        gravity[2] = alpha * gravity[2] + (1 - alpha) * accel[2];
        linAc[0] = accel[0] - gravity[0];
        linAc[1] = accel[1] - gravity[1];
        linAc[2] = accel[2] - gravity[2];*/

        // Коэффициенты, выбранные для вертикально расположенного конкретно моего телефона
        linAc[0] = accel[0] - 0.08717346f;
        linAc[1] = accel[1] - 9.549754f;
        linAc[2] = accel[2] + 0.56110376f;
        double newAc = Math.sqrt(linAc[0]*linAc[0] /*+ linAc[1]*linAc[1]*/ + linAc[2]*linAc[2]);

        // ФНЧ
        float k = 0.05f;
        ac = (1-k)*ac + k * newAc;
        double d = ac * time * time / 2000000;
        distance += d;

        // Задание координат для изображения траектории на плоскости
        TextView text = (TextView)findViewById(R.id.textView);
        text.setText(String.valueOf(distance));
        mDrawView.mX += d * Math.cos(azimuth * Math.PI/180);
        mDrawView.mY += d * Math.sin(azimuth * Math.PI/180);
    }

    // вычисляем вектор поворота по значениям угловых скоростей гироскопа
    private void getRotationVectorFromGyro(float[] gyroValues,
                                           float[] deltaRotationVector,
                                           float timeFactor)
    {
        float[] normValues = new float[3];

        // вычисляем угловую скорость
        float omegaMagnitude =
                (float)Math.sqrt(gyroValues[0] * gyroValues[0] +
                        gyroValues[1] * gyroValues[1] +
                        gyroValues[2] * gyroValues[2]);

        // нормируем вектор вращения, если он достаточно большой, чтобы получить ось
        if(omegaMagnitude > EPSILON) {
            normValues[0] = gyroValues[0] / omegaMagnitude;
            normValues[1] = gyroValues[1] / omegaMagnitude;
            normValues[2] = gyroValues[2] / omegaMagnitude;
        }

        float thetaOverTwo = omegaMagnitude * timeFactor;
        float sinThetaOverTwo = (float)Math.sin(thetaOverTwo);
        float cosThetaOverTwo = (float)Math.cos(thetaOverTwo);
        deltaRotationVector[0] = sinThetaOverTwo * normValues[0];
        deltaRotationVector[1] = sinThetaOverTwo * normValues[1];
        deltaRotationVector[2] = sinThetaOverTwo * normValues[2];
        deltaRotationVector[3] = cosThetaOverTwo;
    }

    // эта функция интегрирует данные гироскопа
    // ориентация с гироскопа пишется в gyroOrientation
    public void gyroFunction(SensorEvent event) {
        // не начинать, пока не получены первые данные с акселерометра/магнетометра
        if (accMagOrientation == null)
            return;

        // инициализация матрицы поворота на основе гироскопа
        if(initState) {
            float[] initMatrix = new float[9];
            initMatrix = getRotationMatrixFromOrientation(accMagOrientation);
            float[] test = new float[3];
            SensorManager.getOrientation(initMatrix, test);
            gyroMatrix = matrixMultiplication(gyroMatrix, initMatrix);
            initState = false;
        }

        // копируем новые данные с гироскопа в массив gyro
        // конвертируем "сырые" данные с гироскопа в вектор поворота
        float[] deltaVector = new float[4];
        if(timestamp != 0) {
            final float dT = (event.timestamp - timestamp) * NS2S;
            System.arraycopy(event.values, 0, gyro, 0, 3);
            getRotationVectorFromGyro(gyro, deltaVector, dT / 2.0f);
        }

        // измерение завершено, сохраняем время для следующего интервала
        timestamp = event.timestamp;

        // конвертируем вектор поворота в матрицу поворота
        float[] deltaMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector);

        // применяем новый интервал поворота на матрице поворота, полученной по данным гироскопа
        gyroMatrix = matrixMultiplication(gyroMatrix, deltaMatrix);

        // берем ориентацию на основе гироскопа из матрицы поворота
        SensorManager.getOrientation(gyroMatrix, gyroOrientation);
    }

    private float[] getRotationMatrixFromOrientation(float[] o) {
        float[] xM = new float[9];
        float[] yM = new float[9];
        float[] zM = new float[9];

        float sinX = (float)Math.sin(o[1]);
        float cosX = (float)Math.cos(o[1]);
        float sinY = (float)Math.sin(o[2]);
        float cosY = (float)Math.cos(o[2]);
        float sinZ = (float)Math.sin(o[0]);
        float cosZ = (float)Math.cos(o[0]);

        // вращение вокруг оси x (pitch)
        xM[0] = 1.0f; xM[1] = 0.0f; xM[2] = 0.0f;
        xM[3] = 0.0f; xM[4] = cosX; xM[5] = sinX;
        xM[6] = 0.0f; xM[7] = -sinX; xM[8] = cosX;

        // вращение вокруг оси y(roll)
        yM[0] = cosY; yM[1] = 0.0f; yM[2] = sinY;
        yM[3] = 0.0f; yM[4] = 1.0f; yM[5] = 0.0f;
        yM[6] = -sinY; yM[7] = 0.0f; yM[8] = cosY;

        // вращение вокруг оси z(azimuth)
        zM[0] = cosZ; zM[1] = sinZ; zM[2] = 0.0f;
        zM[3] = -sinZ; zM[4] = cosZ; zM[5] = 0.0f;
        zM[6] = 0.0f; zM[7] = 0.0f; zM[8] = 1.0f;

        // порядок вращения y, x, z (roll, pitch, azimuth)
        float[] resultMatrix = matrixMultiplication(xM, yM);
        resultMatrix = matrixMultiplication(zM, resultMatrix);
        return resultMatrix;
    }

    private float[] matrixMultiplication(float[] A, float[] B) {
        float[] result = new float[9];

        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

        return result;
    }

    @Override
    public void onClick(View v) {
        String cid = v.getResources().getResourceName(v.getId());
        Log.d("cid", cid);
        if(cid.equals("intelligence.dummy.dip:id/button")) {
            mDrawView.setVisibility(View.INVISIBLE);
            mDrawView.restart();
            sw.setChecked(true);
            distance = 0;
            oldTime = 0;
            ac = 0;
            mDrawView.setVisibility(View.VISIBLE);
        }
        else if (cid.equals("intelligence.dummy.dip:id/button2") && !granted) {
            Activity activity = (Activity)v.getContext();
            int permission = ActivityCompat.checkSelfPermission(activity, Manifest.permission.WRITE_EXTERNAL_STORAGE);
            if (permission != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(
                        activity,
                        PERMISSIONS_STORAGE,
                        REQUEST_EXTERNAL_STORAGE
                );
            }
            else {
                granted = !granted;
                try {
                    File root = android.os.Environment.getExternalStorageDirectory();
                    File path = new File(root.getAbsolutePath() + "/ap");
                    path.mkdirs();
                    File file = new File(path, "dip_out");
                    stream = new FileOutputStream(file);
                } catch (FileNotFoundException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    class calculateFusedOrientationTask extends TimerTask {
        public void run() {
            float oneMinusCoeff = 1.0f - FILTER_COEFFICIENT;

            /*
             * Исправление для проблемы перехода 179° <--> -179°:
             * Если одни из углов ориентации (gyro или accMag) отрицательный, когда другой
             * положительный, добавить 360° (2 * math.PI) к отрицательному значению, произвести
             * sensor fusion, и вычесть 360° из результата, если он больше 180°.
             */

            // рысканье(azimuth/yaw)
            if (gyroOrientation[0] < -0.5 * Math.PI && accMagOrientation[0] > 0.0) {
                fusedOrientation[0] = (float) (FILTER_COEFFICIENT * (gyroOrientation[0] + 2.0 * Math.PI)
                        + oneMinusCoeff * accMagOrientation[0]);
                fusedOrientation[0] -= (fusedOrientation[0] > Math.PI) ? 2.0 * Math.PI : 0;
            }
            else if (accMagOrientation[0] < -0.5 * Math.PI && gyroOrientation[0] > 0.0) {
                fusedOrientation[0] = (float) (FILTER_COEFFICIENT * gyroOrientation[0]
                        + oneMinusCoeff * (accMagOrientation[0] + 2.0 * Math.PI));
                fusedOrientation[0] -= (fusedOrientation[0] > Math.PI)? 2.0 * Math.PI : 0;
            }
            else {
                fusedOrientation[0] = FILTER_COEFFICIENT * gyroOrientation[0]
                        + oneMinusCoeff * accMagOrientation[0];
            }

            // уклон(pitch)
            if (gyroOrientation[1] < -0.5 * Math.PI && accMagOrientation[1] > 0.0) {
                fusedOrientation[1] = (float) (FILTER_COEFFICIENT * (gyroOrientation[1] + 2.0 * Math.PI)
                        + oneMinusCoeff * accMagOrientation[1]);
                fusedOrientation[1] -= (fusedOrientation[1] > Math.PI) ? 2.0 * Math.PI : 0;
            }
            else if (accMagOrientation[1] < -0.5 * Math.PI && gyroOrientation[1] > 0.0) {
                fusedOrientation[1] = (float) (FILTER_COEFFICIENT * gyroOrientation[1]
                        + oneMinusCoeff * (accMagOrientation[1] + 2.0 * Math.PI));
                fusedOrientation[1] -= (fusedOrientation[1] > Math.PI)? 2.0 * Math.PI : 0;
            }
            else {
                fusedOrientation[1] = FILTER_COEFFICIENT * gyroOrientation[1]
                        + oneMinusCoeff * accMagOrientation[1];
            }

            // крен(roll)
            if (gyroOrientation[2] < -0.5 * Math.PI && accMagOrientation[2] > 0.0) {
                fusedOrientation[2] = (float) (FILTER_COEFFICIENT * (gyroOrientation[2] + 2.0 * Math.PI)
                        + oneMinusCoeff * accMagOrientation[2]);
                fusedOrientation[2] -= (fusedOrientation[2] > Math.PI) ? 2.0 * Math.PI : 0;
            }
            else if (accMagOrientation[2] < -0.5 * Math.PI && gyroOrientation[2] > 0.0) {
                fusedOrientation[2] = (float) (FILTER_COEFFICIENT * gyroOrientation[2]
                        + oneMinusCoeff * (accMagOrientation[2] + 2.0 * Math.PI));
                fusedOrientation[2] -= (fusedOrientation[2] > Math.PI)? 2.0 * Math.PI : 0;
            }
            else {
                fusedOrientation[2] = FILTER_COEFFICIENT * gyroOrientation[2]
                        + oneMinusCoeff * accMagOrientation[2];
            }

            // перезапись матрицы гироскопа и ориентации ориенатцией с sensor fusion
            // для компенсации дрейфа гироскопа
            gyroMatrix = getRotationMatrixFromOrientation(fusedOrientation);
            System.arraycopy(fusedOrientation, 0, gyroOrientation, 0, 3);


            // обновление показаний датчиков в пользовательском интерфейсе
            mHandler.post(updateOrientationDisplayTask);
        }
    }

    // функции пользовательского интерфейса

    @Override
    public void onCheckedChanged (RadioGroup group, int checkedId) {
        switch(checkedId) {
            case R.id.radio0:
                radioSelection1 = 0;
                break;
            case R.id.radio1:
                radioSelection1 = 1;
                break;
            case R.id.radio2:
                radioSelection1 = 2;
                break;
            case R.id.radio3:
                radioSelection2 = 0;
                break;
            case R.id.radio4:
                radioSelection2 = 1;
                break;
            case R.id.radio5:
                radioSelection2 = 2;
                break;
        }
    }

    @Override
    public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
        if(isChecked) {
            plot.setVisibility(View.INVISIBLE);
            mDrawView.setVisibility(View.VISIBLE);
        }
        else {
            plot.setVisibility(View.VISIBLE);
            mDrawView.setVisibility(View.INVISIBLE);
        }
    }

    public void updateOrientationDisplay() {
        double pitch, roll;
        double amAzimuth, amPitch, amRoll, gAzimuth, gPitch, gRoll, fAzimuth, fPitch, fRoll;
        azimuth = pitch = roll = 0;
        amAzimuth = accMagOrientation[0] * 180/Math.PI;
        amPitch = accMagOrientation[1] * 180/Math.PI;
        amRoll = accMagOrientation[2] * 180/Math.PI;
        gAzimuth = gyroOrientation[0] * 180/Math.PI;
        gPitch = gyroOrientation[1] * 180/Math.PI;
        gRoll = gyroOrientation[2] * 180/Math.PI;
        fAzimuth = fusedOrientation[0] * 180/Math.PI;
        fPitch = fusedOrientation[1] * 180/Math.PI;
        fRoll = fusedOrientation[2] * 180/Math.PI;
        if(granted) {
            try {
                stream.write(("TIM\t" + System.currentTimeMillis()
                        + "\nAMA\t" + amAzimuth + "\nAMP\t" + amPitch + "\nAMR\t" + amRoll
                        + "\nGA\t" + gAzimuth + "\nGP\t" + gPitch + "\nGR\t" + gRoll
                        + "\nFA\t" + fAzimuth + "\nFP\t" + fPitch + "\nFR\t" + fRoll + "\n").getBytes());
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            } catch (IOException e) {
                Log.d("diplom_log", "Не могу записать");
                e.printStackTrace();
            }
        }
        switch(radioSelection1) {
           case 0:
               azimuth = amAzimuth;
               pitch = amPitch;
               roll = amRoll;
               break;
           case 1:
               azimuth = gAzimuth;
               pitch = gPitch;
               roll = gRoll;
               break;
           case 2:
               azimuth = fAzimuth;
               pitch = fPitch;
               roll = fRoll;
               break;
        }
        shift(azimuth, pitch, roll);
        switch(radioSelection2) {
            case 0:
                plot.setTitle("Azimuth");
                series1 = new SimpleXYSeries(Arrays.asList(seriesAzimuth),
                        SimpleXYSeries.ArrayFormat.Y_VALS_ONLY, "Series1");
                break;
            case 1:
                plot.setTitle("Pitch");
                series1 = new SimpleXYSeries(Arrays.asList(seriesPitch),
                        SimpleXYSeries.ArrayFormat.Y_VALS_ONLY, "Series1");
                break;
            case 2:
                plot.setTitle("Roll");
                series1 = new SimpleXYSeries(Arrays.asList(seriesRoll),
                        SimpleXYSeries.ArrayFormat.Y_VALS_ONLY, "Series1");
                break;
            default:
                plot.setTitle("Azimuth");
                series1 = new SimpleXYSeries(Arrays.asList(seriesAzimuth),
                        SimpleXYSeries.ArrayFormat.Y_VALS_ONLY, "Series1");
                break;
        }
        plot.clear();
        plot.addSeries(series1, series1Format);
        plot.redraw();
   }

    private Runnable updateOrientationDisplayTask = new Runnable() {
        public void run() {
            updateOrientationDisplay();
        }
    };

    private void shift(double a, double p, double r) {
        // перестраиваем массивы отображаемых точек
        seriesAzimuth = Arrays.copyOfRange(seriesAzimuth,1,seriesAzimuth.length);
        seriesAzimuth = Arrays.copyOf(seriesAzimuth, seriesAzimuth.length + 1);
        seriesAzimuth[seriesAzimuth.length-1] = a;
        seriesPitch = Arrays.copyOfRange(seriesPitch,1,seriesPitch.length);
        seriesPitch = Arrays.copyOf(seriesPitch, seriesPitch.length + 1);
        seriesPitch[seriesPitch.length-1] = p;
        seriesRoll = Arrays.copyOfRange(seriesRoll,1,seriesRoll.length);
        seriesRoll = Arrays.copyOf(seriesRoll, seriesRoll.length + 1);
        seriesRoll[seriesRoll.length-1] = r;
    }
}