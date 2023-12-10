package com.john1q.gamepad_control;

import android.app.Application;

public class MyApplication extends Application
{
    private static MyApplication sInstance;
    ConnectedThread connectedThread = null;

    public static MyApplication getApplication() {
        return sInstance;
    }

    public  void setupConnectedThread() {
    }

    public void onCreate() {
        super.onCreate();
        sInstance = this;
    }

    public void setupConnectedThread(ConnectedThread connectedThread)
    {
        this.connectedThread=connectedThread;
    }

    public ConnectedThread getCurrentConnectedThread()
    {
        return connectedThread;
    }
}