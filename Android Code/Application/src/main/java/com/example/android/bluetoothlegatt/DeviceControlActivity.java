/*
 * Copyright (C) 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.example.android.bluetoothlegatt;

import android.app.Activity;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.IBinder;
import android.os.PowerManager;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.CompoundButton;
import android.widget.ExpandableListView;
import android.widget.ImageView;
import android.widget.SeekBar;
import android.widget.SimpleExpandableListAdapter;
import android.widget.Switch;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * For a given BLE device, this Activity provides the user interface to connect, display data,
 * and display GATT services and characteristics supported by the device.  The Activity
 * communicates with {@code BluetoothLeService}, which in turn interacts with the
 * Bluetooth LE API.
 */
public class DeviceControlActivity extends Activity implements SeekBar.OnSeekBarChangeListener{
    private final static String TAG = DeviceControlActivity.class.getSimpleName();

    public static final String EXTRAS_DEVICE_NAME = "DEVICE_NAME";
    public static final String EXTRAS_DEVICE_ADDRESS = "DEVICE_ADDRESS";

    public static String CLIENT_CHARACTERISTIC_MOTOR = "00002a39-0000-1000-8000-00805f9b34fb";

    private String mDeviceName;
    private String mDeviceAddress;
    private BluetoothLeService mBluetoothLeService;
    private boolean mConnected = false;

    private final String LIST_NAME = "NAME";
    private final String LIST_UUID = "UUID";

    private byte[] value = new byte[4];

    private boolean motor_run = false;
    private TextView mShowSpeed0,mShowSpeed1,mShowSpeed2,mShowSpeed3;
    private SeekBar mSetSpeed0,mSetSpeed1,mSetSpeed2,mSetSpeed3;
    private BluetoothGattCharacteristic mMotorChars;
    private ImageView mLedColor;
    private Bitmap    mBitmap;
    private int       MapWidth;
    private int       MapWHeight;
    private int       color_w = 0;
    private int       color_b = 0;
    private int       color_g = 0;
    private int       color_r = 0;

    private PowerManager pm = null;
    PowerManager.WakeLock wl = null;

    // Code to manage Service lifecycle.
    private final ServiceConnection mServiceConnection = new ServiceConnection() {

        @Override
        public void onServiceConnected(ComponentName componentName, IBinder service) {
            mBluetoothLeService = ((BluetoothLeService.LocalBinder) service).getService();
            if (!mBluetoothLeService.initialize()) {
                Log.e(TAG, "Unable to initialize Bluetooth");
                finish();
            }
            // Automatically connects to the device upon successful start-up initialization.
            mBluetoothLeService.connect(mDeviceAddress);
        }

        @Override
        public void onServiceDisconnected(ComponentName componentName) {
            mBluetoothLeService = null;
        }
    };

    // Handles various events fired by the Service.
    // ACTION_GATT_CONNECTED: connected to a GATT server.
    // ACTION_GATT_DISCONNECTED: disconnected from a GATT server.
    // ACTION_GATT_SERVICES_DISCOVERED: discovered GATT services.
    // ACTION_DATA_AVAILABLE: received data from the device.  This can be a result of read
    //                        or notification operations.
    private final BroadcastReceiver mGattUpdateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            if (BluetoothLeService.ACTION_GATT_CONNECTED.equals(action)) {
                mConnected = true;
                invalidateOptionsMenu();
            } else if (BluetoothLeService.ACTION_GATT_DISCONNECTED.equals(action)) {
                mConnected = false;
                invalidateOptionsMenu();
                clearUI();
            } else if (BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED.equals(action)) {
                // Show all the supported services and characteristics on the user interface.
                displayGattServices(mBluetoothLeService.getSupportedGattServices());
            } else if (BluetoothLeService.ACTION_DATA_AVAILABLE.equals(action)) {
            }
        }
    };

    private void clearUI() {
        mBluetoothLeService = null;
        Intent intent = new Intent(this, DeviceScanActivity.class);
        startActivity(intent);
        //finish();
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.gatt_services_characteristics);

        mShowSpeed0 = (TextView) findViewById(R.id.ShowSpeed0);
        mSetSpeed0  = (SeekBar) findViewById(R.id.SetSpeed0);
        mShowSpeed1 = (TextView) findViewById(R.id.ShowSpeed1);
        mSetSpeed1  = (SeekBar) findViewById(R.id.SetSpeed1);
        mShowSpeed2 = (TextView) findViewById(R.id.ShowSpeed2);
        mSetSpeed2  = (SeekBar) findViewById(R.id.SetSpeed2);
        mShowSpeed3 = (TextView) findViewById(R.id.ShowSpeed3);
        mSetSpeed3  = (SeekBar) findViewById(R.id.SetSpeed3);
        mLedColor   = (ImageView)findViewById(R.id.LEDColorView);

        // Update Current PWM Chx Value
        mShowSpeed0.setText("White PWM Duty " + 0);
        mSetSpeed0.setMax(100);
        mSetSpeed0.setProgress(0);
        mSetSpeed0.setOnSeekBarChangeListener(this);

        mShowSpeed1.setText("Blue PWM Duty " + 0);
        mSetSpeed1.setMax(100);
        mSetSpeed1.setProgress(0);
        mSetSpeed1.setOnSeekBarChangeListener(this);

        mShowSpeed2.setText("Green PWM Duty " + 0);
        mSetSpeed2.setMax(100);
        mSetSpeed2.setProgress(0);
        mSetSpeed2.setOnSeekBarChangeListener(this);

        mShowSpeed3.setText("Red PWM Duty " + 0);
        mSetSpeed3.setMax(100);
        mSetSpeed3.setProgress(0);
        mSetSpeed3.setOnSeekBarChangeListener(this);

        final Intent intent = getIntent();
        mDeviceName = intent.getStringExtra(EXTRAS_DEVICE_NAME);
        mDeviceAddress = intent.getStringExtra(EXTRAS_DEVICE_ADDRESS);

        // Sets up UI references.
        getActionBar().setTitle(mDeviceName);
        getActionBar().setDisplayHomeAsUpEnabled(true);
        Intent gattServiceIntent = new Intent(this, BluetoothLeService.class);
        bindService(gattServiceIntent, mServiceConnection, BIND_AUTO_CREATE);

    }

    @Override
    public void onWindowFocusChanged(boolean focus) {
        super.onWindowFocusChanged(focus);
        Log.d(TAG, "Measure Called");
        if(null == mBitmap) {
            mBitmap = Bitmap.createBitmap(mLedColor.getWidth()/8, mLedColor.getHeight()/8, Bitmap.Config.ARGB_8888);

            MapWHeight = mLedColor.getHeight()/8;
            MapWidth = mLedColor.getWidth()/8;

            for (int i = 0; i < MapWidth; i++) {
                for (int j = 0; j < MapWHeight / 2; j++) {
                    mBitmap.setPixel(i, j, Color.argb(255, color_w, color_w, color_w));
                }

                for (int j = MapWHeight / 2; j < MapWHeight; j++) {
                    mBitmap.setPixel(i, j, Color.argb(255, color_r, color_g, color_b));
                }
            }

            mLedColor.setImageBitmap(mBitmap);
            //return;
        }
    }

    @Override
    public void onProgressChanged(SeekBar var1, int var2, boolean var3) {

        if(mSetSpeed0 == var1) {
            color_w = var1.getProgress();
            value[0] = (byte) color_w;
            mShowSpeed0.setText("White PWM Duty " + var1.getProgress());
        }
        if(mSetSpeed1 == var1) {
            color_b = var1.getProgress();
            value[1] = (byte) color_b;
            mShowSpeed1.setText("Blue PWM Duty " + var1.getProgress());
        }
        if(mSetSpeed2 == var1) {
            color_g = var1.getProgress();
            value[2] = (byte) color_g;
            mShowSpeed2.setText("Green PWM Duty " + var1.getProgress());
        }
        if(mSetSpeed3 == var1) {
            color_r = var1.getProgress();
            value[3] = (byte) color_r;
            mShowSpeed3.setText("Red PWM Duty " + var1.getProgress());
        }

        if(null != mMotorChars) {
            mMotorChars.setValue(value);
            mBluetoothLeService.writeCharacteristic(mMotorChars);
        }

        if(null != mLedColor)
        {
            if(null != mBitmap) {
                int _color_w = color_w*2 + color_w/2;
                int _color_r = color_r*2 + color_r/2;
                int _color_g = color_g*2 + color_g/2;
                int _color_b = color_b*2 + color_b/2;

                for (int i = 0; i < MapWidth; i++) {
                    for (int j = 0; j < MapWHeight / 2; j++) {
                        mBitmap.setPixel(i, j, Color.argb(255, _color_w, _color_w, _color_w));
                    }

                    for (int j = MapWHeight / 2; j < MapWHeight; j++) {
                        mBitmap.setPixel(i, j, Color.argb(255, _color_r, _color_g, _color_b));
                    }
                }
            }

        }
    }

    @Override
    public void onStartTrackingTouch(SeekBar var1){

    }

    @Override
    public void onStopTrackingTouch(SeekBar var1){

    }

    @Override
    protected void onResume() {
        super.onResume();
        if(null == pm) {
            pm = (PowerManager) getSystemService(Context.POWER_SERVICE);
            wl = pm.newWakeLock(PowerManager.SCREEN_BRIGHT_WAKE_LOCK, "My Tag");
            wl.acquire();
        }

        registerReceiver(mGattUpdateReceiver, makeGattUpdateIntentFilter());
        if (mBluetoothLeService != null) {
            final boolean result = mBluetoothLeService.connect(mDeviceAddress);
            Log.d(TAG, "Connect request result=" + result);
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        mBluetoothLeService.disconnect();
        unregisterReceiver(mGattUpdateReceiver);
        if(null != wl) {
            wl.release();
            wl = null;
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        unbindService(mServiceConnection);
        mBluetoothLeService = null;
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.gatt_services, menu);
        if (mConnected) {
            menu.findItem(R.id.menu_connect).setVisible(false);
            menu.findItem(R.id.menu_disconnect).setVisible(true);
        } else {
            menu.findItem(R.id.menu_connect).setVisible(true);
            menu.findItem(R.id.menu_disconnect).setVisible(false);
        }
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch(item.getItemId()) {
            case R.id.menu_connect:
                mBluetoothLeService.connect(mDeviceAddress);
                return true;
            case R.id.menu_disconnect:
                mBluetoothLeService.disconnect();
                return true;
            case android.R.id.home:
                onBackPressed();
                return true;
        }
        return super.onOptionsItemSelected(item);
    }

    private static IntentFilter makeGattUpdateIntentFilter() {
        final IntentFilter intentFilter = new IntentFilter();
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_CONNECTED);
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_DISCONNECTED);
        intentFilter.addAction(BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED);
        intentFilter.addAction(BluetoothLeService.ACTION_DATA_AVAILABLE);
        return intentFilter;
    }

    // Demonstrates how to iterate through the supported GATT Services/Characteristics.
    // In this sample, we populate the data structure that is bound to the ExpandableListView
    // on the UI.
    private void displayGattServices(List<BluetoothGattService> gattServices) {
        if (gattServices == null) return;
        String uuid = null;
        String unknownServiceString = getResources().getString(R.string.unknown_service);
        String unknownCharaString = getResources().getString(R.string.unknown_characteristic);
        ArrayList<HashMap<String, String>> gattServiceData = new ArrayList<HashMap<String, String>>();

        // Loops through available GATT Services.
        for (BluetoothGattService gattService : gattServices) {
            HashMap<String, String> currentServiceData = new HashMap<String, String>();
            uuid = gattService.getUuid().toString();
            currentServiceData.put(
                    LIST_NAME, SampleGattAttributes.lookup(uuid, unknownServiceString));
            currentServiceData.put(LIST_UUID, uuid);
            gattServiceData.add(currentServiceData);

            ArrayList<HashMap<String, String>> gattCharacteristicGroupData =
                    new ArrayList<HashMap<String, String>>();
            List<BluetoothGattCharacteristic> gattCharacteristics =
                    gattService.getCharacteristics();
            ArrayList<BluetoothGattCharacteristic> charas =
                    new ArrayList<BluetoothGattCharacteristic>();

            // Loops through available Characteristics.
            for (BluetoothGattCharacteristic gattCharacteristic : gattCharacteristics) {
                charas.add(gattCharacteristic);
                uuid = gattCharacteristic.getUuid().toString();
                Log.d(TAG, uuid);

                // Set
                if(gattCharacteristic.getUuid().toString().equals(CLIENT_CHARACTERISTIC_MOTOR)) {
                    mMotorChars = gattCharacteristic;
                    Log.d(TAG, "Find it");
                }
            }
        }
    }

}
