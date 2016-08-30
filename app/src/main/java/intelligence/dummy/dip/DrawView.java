package intelligence.dummy.dip;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

public class DrawView extends SurfaceView implements SurfaceHolder.Callback {

    public float mX, mY, mXold, mYold;
    private DrawThread drawThread;
    private Paint paintTrack, paintBack, paintCurrentPosition;
    private boolean start;
    private static final float k = 1000;

    public DrawView(Context context) {
        super(context);
        init();
    }

    public DrawView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public void restart() {
        start = false;
    }

    private void init() {
        getHolder().addCallback(this);
        paintBack = new Paint();
        paintBack.setStyle(Paint.Style.FILL);
        paintBack.setColor(Color.BLACK);
        paintTrack = new Paint();
        paintTrack.setStyle(Paint.Style.FILL);
        paintTrack.setColor(Color.WHITE);
        paintCurrentPosition = new Paint();
        paintCurrentPosition.setStyle(Paint.Style.FILL);
        paintCurrentPosition.setColor(Color.BLUE);
        start = false;
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        drawThread = new DrawThread(getHolder());
        drawThread.setRunning(true);
        drawThread.start();
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        boolean retry = true;
        drawThread.setRunning(false);
        while (retry) {
            try {
                drawThread.join();
                retry = false;
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    class DrawThread extends Thread {

        private boolean running = false;
        private SurfaceHolder surfaceHolder;

        public DrawThread(SurfaceHolder surfaceHolder) {
            this.surfaceHolder = surfaceHolder;
        }

        public void setRunning(boolean running) {
            this.running = running;
        }

        @Override
        public void run() {
            Canvas canvas;
            while (running) {
                canvas = null;
                try {
                    canvas = surfaceHolder.lockCanvas(null);
                    if (canvas == null)
                        continue;
                    draw(canvas);
                } finally {
                    if (canvas != null) {
                        surfaceHolder.unlockCanvasAndPost(canvas);
                    }
                }
            }
        }

        private void draw(Canvas canvas) {
            if(!start) {
                canvas.drawColor(Color.BLACK);
                mY = 0;
                mX = 0;
                mXold = 0;
                mYold = 0;
                start = true;
            }
            canvas.drawCircle(mXold* k +getWidth()/2, mYold* k +getHeight()/2, 3, paintTrack);
            canvas.drawCircle(mX* k +getWidth()/2, mY* k +getHeight()/2, 2, paintCurrentPosition);
            mXold = mX;
            mYold = mY;
        }
    }
}