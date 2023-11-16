package com.john1q.gamepad_control;

import static androidx.core.view.MotionEventCompat.getAxisValue;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.view.InputDevice;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.widget.SeekBar;
import android.widget.TextView;
import android.os.Handler;

//import com.sarmale.arduinobtexampleledcontrol.R;
import com.john1q.gamepad_control.R;

public class ConfigureLed extends AppCompatActivity {

    private TextView textView;
    private TextView textView2;
    private TextView textView3;
    private SeekBar seekBar;
    Handler handler = new Handler();
    Runnable runnable;
    int delay = 25;

    int stick_x, stick_y, btnx, btny, btnb, btna, r2, l2 = 0;

    int joystick_connected;

    String message = String.valueOf(stick_x) + "." + String.valueOf(stick_y) + "." + String.valueOf(r2) + "." +
            String.valueOf(l2) + "." +String.valueOf(btna) + "." + String.valueOf(btnb) + "." +
            String.valueOf(btnx) + "." + String.valueOf(btny);

    ConnectedThread connectedThread;

    private static final String TAG = "FrugalLogs";


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_configure_led);

        textView = findViewById(R.id.textView);
        textView2 = findViewById(R.id.textView2);
        textView3 = findViewById(R.id.textView3);
        seekBar = findViewById(R.id.seekBar);

        connectedThread = MyApplication.getApplication().getCurrentConnectedThread();
    }

    @Override
    protected void onResume() {
        handler.postDelayed(runnable = new Runnable() {
            public void run() {
                handler.postDelayed(runnable, delay);
                // send a text: 'a'
                connectedThread.write(message + "\r\n");
                message = String.valueOf(stick_x) + "." + String.valueOf(stick_y) + "." + String.valueOf(r2) + "." +
                        String.valueOf(l2) + "." +String.valueOf(btna) + "." + String.valueOf(btnb) + "." +
                        String.valueOf(btnx) + "." + String.valueOf(btny);

            }
        }, delay);
        super.onResume();
    }
    @Override
    protected void onPause() {
        super.onPause();
        handler.removeCallbacks(runnable); //stop handler when activity not visible super.onPause();
    }



    @Override
    public boolean dispatchGenericMotionEvent(MotionEvent event) {
        if ((event.getSource() & InputDevice.SOURCE_JOYSTICK) == InputDevice.SOURCE_JOYSTICK &&
                event.getAction() == MotionEvent.ACTION_MOVE) {
            // Get X and Y axis values of the joystick
            float xValue = getCenteredAxis(event, MotionEvent.AXIS_X);
            float yValue = getCenteredAxis(event, MotionEvent.AXIS_Y);

            float r2_val = event.getAxisValue(MotionEvent.AXIS_RTRIGGER);
            float l2_val = event.getAxisValue(MotionEvent.AXIS_LTRIGGER);

            stick_x = (int) (xValue * 100);
            stick_y = (int) (yValue * 100);

            r2 = (int) (r2_val * 100);
            l2 = (int) (l2_val * 100);

            // Use the X and Y values as needed (here, just displaying in TextView)
            textView2.setText("Joystick X: " + stick_x + "\nJoystick Y: " + stick_y + "\nL2: " + l2 + "\nR2: " + r2);

            message = String.valueOf(stick_x) + "." + String.valueOf(stick_y) + "." + String.valueOf(r2) + "." +
                    String.valueOf(l2) + "." +String.valueOf(btna) + "." + String.valueOf(btnb) + "." +
                    String.valueOf(btnx) + "." + String.valueOf(btny);

            textView3.setText(message);



            return true; // To consume the event

        }

        return super.dispatchGenericMotionEvent(event);
    }

    private static float getCenteredAxis(MotionEvent event, int axis) {
        InputDevice device = event.getDevice();
        float flat = 0.5f;
        float value = event.getAxisValue(axis);
        final float deadZone = device.getMotionRange(axis).getFlat();
        if (Math.abs(value) > deadZone) {
            if (value > 0) {
                return Math.min(value, 1);
            } else {
                return Math.max(value, -1);
            }
        }
        return 0;
    }

    @Override
    public boolean dispatchKeyEvent(KeyEvent event) {
        int keyCode = event.getKeyCode();

        if (event.getAction() == KeyEvent.ACTION_DOWN && keyCode == KeyEvent.KEYCODE_BUTTON_X) {
            // Change TextView value when "X" button is pressed
            textView.setText("Button X Pressed!");
            btnx = 1;

            message = String.valueOf(stick_x) + "." + String.valueOf(stick_y) + "." + String.valueOf(r2) + "." +
                    String.valueOf(l2) + "." +String.valueOf(btna) + "." + String.valueOf(btnb) + "." +
                    String.valueOf(btnx) + "." + String.valueOf(btny);

            textView3.setText(message);
            // SHOUD HERE BE THE b \r\n??
            return true; // To consume the event
        } else {
            btnx = 0;
        }

        if (event.getAction() == KeyEvent.ACTION_DOWN && keyCode == KeyEvent.KEYCODE_BUTTON_Y) {
            // Change TextView value when "X" button is pressed
            textView.setText("Button Y Pressed!");
            btny = 1;

            message = String.valueOf(stick_x) + "." + String.valueOf(stick_y) + "." + String.valueOf(r2) + "." +
                    String.valueOf(l2) + "." +String.valueOf(btna) + "." + String.valueOf(btnb) + "." +
                    String.valueOf(btnx) + "." + String.valueOf(btny);

            textView3.setText(message);
            return true; // To consume the event
        } else {
            btny = 0;
        }

        if (event.getAction() == KeyEvent.ACTION_DOWN && keyCode == KeyEvent.KEYCODE_BUTTON_B) {
            // Change TextView value when "X" button is pressed
            textView.setText("Button B Pressed!");
            btnb = 1;

            message = String.valueOf(stick_x) + "." + String.valueOf(stick_y) + "." + String.valueOf(r2) + "." +
                    String.valueOf(l2) + "." +String.valueOf(btna) + "." + String.valueOf(btnb) + "." +
                    String.valueOf(btnx) + "." + String.valueOf(btny);

            textView3.setText(message);
            return true; // To consume the event
        } else {
            btnb = 0;
        }

        if (event.getAction() == KeyEvent.ACTION_DOWN && keyCode == KeyEvent.KEYCODE_BUTTON_A) {
            // Change TextView value when "X" button is pressed
            textView.setText("Button A Pressed!");
            btna = 1;

            message = String.valueOf(stick_x) + "." + String.valueOf(stick_y) + "." + String.valueOf(r2) + "." +
                    String.valueOf(l2) + "." +String.valueOf(btna) + "." + String.valueOf(btnb) + "." +
                    String.valueOf(btnx) + "." + String.valueOf(btny);

            textView3.setText(message);
            return true; // To consume the event
        } else {
            btna = 0;
        }




        //connectedThread = MyApplication.getApplication().getCurrentConnectedThread();

        return super.dispatchKeyEvent(event);


    }
}



//    private static final String TAG = "FrugalLogs";
//    TextView tvProgressLabelRed,tvProgressLabelGreen,tvProgressLabelBlue;
//    static final  char RED='R';
//    static final  char GREEN='G';
//    static final  char BLUE='B';
//    int redNumber, greenNumber, blueNumber;
//    ConnectedThread connectedThread;
//    @Override
//    protected void onCreate(Bundle savedInstanceState) {
//        super.onCreate(savedInstanceState);
//        setContentView(R.layout.activity_configure_led);
//        SeekBar seekBarRed = findViewById(R.id.seekBarRed);
//        SeekBar seekBarGreen = findViewById(R.id.seekBarGreen);
//        SeekBar seekBarBlue = findViewById(R.id.seekBarBlue);
//        ImageView colorDemo = findViewById(R.id.colorDemo);
//        redNumber=greenNumber=blueNumber=0;
//        colorDemo.setBackgroundColor(Color.rgb(redNumber, greenNumber, blueNumber));
//        tvProgressLabelRed = findViewById(R.id.redValue);
//        tvProgressLabelGreen = findViewById(R.id.greenValue);
//        tvProgressLabelBlue = findViewById(R.id.blueValue);
//        seekBarRed.setTag(RED);
//        seekBarGreen.setTag(GREEN);
//        seekBarBlue.setTag(BLUE);
//        Log.d(TAG, "Launching ConfigureLed Activity");
//
//        // set a change listener on the SeekBar
//        SeekBar.OnSeekBarChangeListener seekBarChangeListener = new SeekBar.OnSeekBarChangeListener() {
//
//            @Override
//            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
//                // updated continuously as the user slides the thumb
//                char color = (char) seekBar.getTag();
//                switch(color) {
//                    case RED:
//                        tvProgressLabelRed.setText("Red: " + progress);
//                        redNumber=progress;
//                        break;
//                    case GREEN:
//                        tvProgressLabelGreen.setText("Green: " + progress);
//                        greenNumber=progress;
//                        break;
//                    case BLUE:
//                        tvProgressLabelBlue.setText("Blue: " + progress);
//                        blueNumber=progress;
//                        break;
//                    default:
//                        throw new IllegalStateException("Unexpected value: " + color);
//                }
//
//                colorDemo.setBackgroundColor(Color.rgb(redNumber, greenNumber, blueNumber));
//
//            }
//            @Override
//            public void onStartTrackingTouch(SeekBar seekBar) {
//                // called when the user first touches the SeekBar
//            }
//
//            @Override
//            public void onStopTrackingTouch(SeekBar seekBar) {
//                // called after the user finishes moving the SeekBar
//                String complete = String.valueOf(redNumber)  + "." + String.valueOf(greenNumber) +"."+String.valueOf(blueNumber);
//
//                connectedThread.write("a");
//
//            }
//        };
//
//        seekBarRed.setOnSeekBarChangeListener(seekBarChangeListener);
//        seekBarGreen.setOnSeekBarChangeListener(seekBarChangeListener);
//        seekBarBlue.setOnSeekBarChangeListener(seekBarChangeListener);
//
//        int progress = seekBarRed.getProgress();
//        tvProgressLabelRed.setText("Red: " + progress);
//        tvProgressLabelGreen.setText("Green: " + progress);
//        tvProgressLabelBlue.setText("Blue: " + progress);
//
//        connectedThread = MyApplication.getApplication().getCurrentConnectedThread();
//        Log.d(TAG, "");
//    }
//}