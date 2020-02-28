package transmissiondemo.dji.com.transmissiondemo;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.net.Uri;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ProgressBar;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import butterknife.BindView;
import butterknife.ButterKnife;
import butterknife.OnClick;
import dji.common.error.DJIError;
import dji.common.error.DJISDKError;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKManager;
import transmissiondemo.dji.com.transmissiondemo.services.SimpleDroneControls;
import transmissiondemo.dji.com.transmissiondemo.services.DroneStatus;
import transmissiondemo.dji.com.transmissiondemo.services.tests.ReceiveBandwidthTestAsync;
import transmissiondemo.dji.com.transmissiondemo.services.tests.SendBandwidthTestAsync;
import transmissiondemo.dji.com.transmissiondemo.services.SendFileAsync;
import transmissiondemo.dji.com.transmissiondemo.utilities.Utilities;

public class MainActivity extends AppCompatActivity {

    @BindView(R.id.progressBarFile)
    public
    ProgressBar progressBarFile;
    @BindView(R.id.buttonRotateLeft)
    ImageButton buttonRotateLeft;
    @BindView(R.id.buttonMoveForward)
    ImageButton buttonMoveForward;
    @BindView(R.id.buttonRotateRight)
    ImageButton buttonRotateRight;
    @BindView(R.id.buttonMoveLeft)
    ImageButton buttonMoveLeft;
    @BindView(R.id.buttonMoveRight)
    ImageButton buttonMoveRight;
    @BindView(R.id.buttonMoveDown)
    Button buttonMoveDown;
    @BindView(R.id.buttonMoveBackward)
    ImageButton buttonMoveBackward;
    @BindView(R.id.buttonMoveUp)
    Button buttonMoveUp;
    @BindView(R.id.buttonSendFile)
    Button buttonSendFile;
    @BindView(R.id.textLogger)
    public
    TextView textLogger;
    @BindView(R.id.buttonTakeOff)
    Button buttonTakeOff;
    @BindView(R.id.buttonFlyTrajectory)
    Button buttonFlyTrajectory;
    @BindView(R.id.buttonLand)
    Button buttonLand;
    @BindView(R.id.buttonHover)
    Button buttonHover;
    @BindView(R.id.textPositionX)
    public
    EditText textPositionX;
    @BindView(R.id.textPositionY)
    public
    EditText textPositionY;
    @BindView(R.id.textPositionZ)
    public
    EditText textPositionZ;
    @BindView(R.id.textLinearVelocityX)
    public
    EditText textLinearVelocityX;
    @BindView(R.id.textLinearVelocityY)
    public
    EditText textLinearVelocityY;
    @BindView(R.id.textLinearVelocityZ)
    public
    EditText textLinearVelocityZ;
    @BindView(R.id.textAngularVelocityX)
    public
    EditText textAngularVelocityX;
    @BindView(R.id.textAngularVelocityY)
    public
    EditText textAngularVelocityY;
    @BindView(R.id.textAngularVelocityZ)
    public
    EditText textAngularVelocityZ;
    @BindView(R.id.textGpsLatitude)
    public
    EditText textGpsLatitude;
    @BindView(R.id.textGpsLongitude)
    public
    EditText textGpsLongitude;
    @BindView(R.id.textGpsAltitude)
    public
    EditText textGpsAltitude;
    @BindView(R.id.textOrientationRoll)
    public
    EditText textOrientationRoll;
    @BindView(R.id.textOrientationPitch)
    public
    EditText textOrientationPitch;
    @BindView(R.id.textOrientationYaw)
    public
    EditText textOrientationYaw;
    @BindView(R.id.buttonSendBandwidthTest)
    Button buttonSendBandwidthTest;
    @BindView(R.id.buttonReceiveBandwidthTest)
    Button buttonReceiveBandwidthTest;

    private SimpleDroneControls droneControls;
    private DroneStatus droneStatus;

    private AsyncTask<Void, Void, Boolean> sendFileAsync;

    private static MainActivity instance;

    private AsyncTask<Void, Void, Boolean> sendBandwidthTestAsync;
    private AsyncTask<Void, Void, Boolean> receiveBandwidthTestAsync;


    private static final String TAG = MainActivity.class.getName();
    private static final String FLAG_CONNECTION_CHANGE = "dji_sdk_connection_change";

    private static final int READ_REQUEST_CODE = 42;

    public final static short DJI_DRONE_SYSID = 1;
    public final static short DJI_DRONE_COMPID = 1;

    public final static short DJI_GROUND_CONTROL_ID = 255;

    private static BaseProduct mProduct;

    private Handler mHandler;
    private static final String[] REQUIRED_PERMISSION_LIST = new String[]{
        Manifest.permission.VIBRATE,
        Manifest.permission.INTERNET,
        Manifest.permission.ACCESS_WIFI_STATE,
        Manifest.permission.WAKE_LOCK,
        Manifest.permission.ACCESS_COARSE_LOCATION,
        Manifest.permission.ACCESS_NETWORK_STATE,
        Manifest.permission.ACCESS_FINE_LOCATION,
        Manifest.permission.CHANGE_WIFI_STATE,
        Manifest.permission.WRITE_EXTERNAL_STORAGE,
        Manifest.permission.BLUETOOTH,
        Manifest.permission.BLUETOOTH_ADMIN,
        Manifest.permission.READ_EXTERNAL_STORAGE,
        Manifest.permission.READ_PHONE_STATE,
    };
    private final List<String> missingPermission = new ArrayList<>();
    private final AtomicBoolean isRegistrationInProgress = new AtomicBoolean(false);
    private static final int REQUEST_PERMISSION_CODE = 12345;

    @Override
    protected void onCreate(final Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        instance = this;

        // When the compile and target version is higher than 22, please request the following permission at runtime to ensure the SDK works well.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            checkAndRequestPermissions();
        } else {
            // DJI Developers forgot something
            // It would be useful, if the app still does something...
            startSDKRegistration();
        }
        setContentView(R.layout.activity_main);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        ButterKnife.bind(this);
        //Initialize DJI SDK Manager
        mHandler = new Handler(Looper.getMainLooper());
    }

    // Should always exist
    public static MainActivity getInstance() {
        return instance;
    }

    public BaseProduct getProduct() {
        return mProduct;
    }

    public static Aircraft getAircraft() {
        return (Aircraft) mProduct;
    }

    public Handler getHandler() {
        return mHandler;
    }

    /**
     * Checks if there is any missing permissions, and
     * requests runtime permission if needed.
     */
    private void checkAndRequestPermissions() {
        // Check for permissions
        for (final String eachPermission : REQUIRED_PERMISSION_LIST) {
            if (ContextCompat.checkSelfPermission(this, eachPermission) != PackageManager.PERMISSION_GRANTED) {
                missingPermission.add(eachPermission);
            }
        }
        // Request for missing permissions
        if (missingPermission.isEmpty()) {
            startSDKRegistration();
        } else if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            Utilities.showToast(MainActivity.this, "Need to grant the permissions!");
            ActivityCompat.requestPermissions(this,
                missingPermission.toArray(new String[missingPermission.size()]),
                REQUEST_PERMISSION_CODE);
        }
    }

    /**
     * Result of runtime permission request
     */
    @Override
    public void onRequestPermissionsResult(final int requestCode,
                                           @NonNull final String[] permissions,
                                           @NonNull final int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        // Check for granted permission and remove from missing list
        if (requestCode == REQUEST_PERMISSION_CODE) {
            for (int i = grantResults.length - 1; i >= 0; i--) {
                if (grantResults[i] == PackageManager.PERMISSION_GRANTED) {
                    missingPermission.remove(permissions[i]);
                }
            }
        }
        // If there is enough permission, we will start the registration
        if (missingPermission.isEmpty()) {
            startSDKRegistration();
        } else {
            Utilities.showToast(MainActivity.this, "Missing permissions!!!");
        }
    }

    private void startSDKRegistration() {
        if (isRegistrationInProgress.compareAndSet(false, true)) {
            AsyncTask.execute(new Runnable() {
                @Override
                public void run() {
                    Utilities.showToast(MainActivity.this, "registering, pls wait...");
                    DJISDKManager.getInstance().registerApp(getApplicationContext(), new DJISDKManager.SDKManagerCallback() {
                        @Override
                        public void onRegister(final DJIError djiError) {
                            if (djiError == DJISDKError.REGISTRATION_SUCCESS) {
                                Utilities.showToast(MainActivity.this, "Register Success");
                                DJISDKManager.getInstance().startConnectionToProduct();
                            } else {
                                Utilities.showToast(MainActivity.this, "Register sdk fails, please check the bundle id and network connection!");
                            }
                            Log.v(TAG, djiError.getDescription());
                        }

                        @Override
                        public void onProductChange(final BaseProduct oldProduct, final BaseProduct newProduct) {
                            mProduct = newProduct;
                            if (mProduct != null && mProduct.getModel() != null) {
                                mProduct.setBaseProductListener(mDJIBaseProductListener);
                                Utilities.showToast(MainActivity.this, "Product name: " + mProduct.getModel().getDisplayName());

                                // START THE CONTROLS
                                droneControls = new SimpleDroneControls();
                                droneStatus = new DroneStatus();
                            } else {
                                Utilities.showToast(MainActivity.this, "Product is null, could be a problem");
                            }

                            notifyStatusChange();
                        }
                    });
                }
            });
        }
    }

    private final BaseProduct.BaseProductListener mDJIBaseProductListener = new BaseProduct.BaseProductListener() {
        @Override
        public void onComponentChange(final BaseProduct.ComponentKey key, final BaseComponent oldComponent, final BaseComponent newComponent) {
            if (newComponent != null) {
                newComponent.setComponentListener(mDJIComponentListener);
            }
            notifyStatusChange();
        }

        @Override
        public void onConnectivityChange(final boolean isConnected) {
            notifyStatusChange();
        }
    };
    private final BaseComponent.ComponentListener mDJIComponentListener = new BaseComponent.ComponentListener() {
        @Override
        public void onConnectivityChange(final boolean isConnected) {
            notifyStatusChange();
        }
    };

    private void notifyStatusChange() {
        mHandler.removeCallbacks(updateRunnable);
        mHandler.postDelayed(updateRunnable, 500);
    }

    private final Runnable updateRunnable = new Runnable() {
        @Override
        public void run() {
            final Intent intent = new Intent(FLAG_CONNECTION_CHANGE);
            sendBroadcast(intent);
        }
    };

    public void stopSendingFile(final View view) {
        if (sendFileAsync != null) {
            sendFileAsync.cancel(true);
        }
    }

    /**
     * Fires an intent to spin up the "file chooser" UI and select an image.
     */
    public void performFileSearch() {

        // ACTION_OPEN_DOCUMENT is the intent to choose a file via the system's file
        // browser.
        final Intent intent = new Intent(Intent.ACTION_GET_CONTENT);

        // Filter to only show results that can be "opened", such as a
        // file (as opposed to a list of contacts or timezones)
        // intent.addCategory(Intent.CATEGORY_OPENABLE);

        intent.addCategory(Intent.CATEGORY_DEFAULT);

        // Filter to show only images, using the image MIME data type.
        // If one wanted to search for ogg vorbis files, the type would be "audio/ogg".
        // To search for all documents available via installed storage providers,
        // it would be "*/*".
        intent.setType("*/*");

        startActivityForResult(intent, READ_REQUEST_CODE);
    }

    @Override
    public void onActivityResult(final int requestCode, final int resultCode,
                                 final Intent resultData) {

        // The ACTION_OPEN_DOCUMENT intent was sent with the request code
        // READ_REQUEST_CODE. If the request code seen here doesn't match, it's the
        // response to some other intent, and the code below shouldn't run at all.

        if (requestCode == READ_REQUEST_CODE && resultCode == Activity.RESULT_OK) {
            // The document selected by the user won't be returned in the intent.
            // Instead, a URI to that document will be contained in the return intent
            // provided to this method as a parameter.
            // Pull that URI using resultData.getData().
            if (resultData != null) {
                final Uri uri = resultData.getData();
                Log.i(TAG, "Uri: " + uri.toString());

                sendFileAsync = new SendFileAsync(uri);
                sendFileAsync.execute();
            }
        }
    }

    @OnClick({R.id.buttonRotateLeft, R.id.buttonMoveForward, R.id.buttonRotateRight, R.id.buttonMoveLeft, R.id.buttonMoveRight, R.id.buttonMoveDown, R.id.buttonMoveBackward, R.id.buttonMoveUp, R.id.buttonSendFile, R.id.buttonTakeOff, R.id.buttonFlyTrajectory, R.id.buttonLand, R.id.buttonHover, R.id.buttonSendBandwidthTest, R.id.buttonReceiveBandwidthTest})
    public void onViewClicked(final View view) {
        switch (view.getId()) {
            case R.id.buttonSendFile:
                performFileSearch();
                break;

            case R.id.buttonRotateLeft:
                droneControls.rotateLeft();
                break;
            case R.id.buttonMoveForward:
                droneControls.moveForward();
                break;
            case R.id.buttonRotateRight:
                droneControls.rotateRight();
                break;
            case R.id.buttonMoveLeft:
                droneControls.moveLeft();
                break;
            case R.id.buttonMoveRight:
                droneControls.moveRight();
                break;
            case R.id.buttonMoveDown:
                droneControls.moveDown();
                break;
            case R.id.buttonMoveBackward:
                droneControls.moveBackward();
                break;
            case R.id.buttonMoveUp:
                droneControls.moveUp();
                break;

            case R.id.buttonTakeOff:
                droneControls.takeOff();
                break;
            case R.id.buttonFlyTrajectory:
                droneControls.flyTrajectory();
                break;
            case R.id.buttonLand:
                droneControls.land();
                break;
            case R.id.buttonHover:
                droneControls.hover();
                break;

            case R.id.buttonSendBandwidthTest:
                if (sendBandwidthTestAsync == null) {
                    sendBandwidthTestAsync = new SendBandwidthTestAsync()
                        .execute();
                    Utilities.showToast("Started SendBandwidthTestAsync");
                } else {
                    sendBandwidthTestAsync.cancel(true);
                    sendBandwidthTestAsync = null;
                    Utilities.showToast("Stopped SendBandwidthTestAsync");
                }
                break;

            case R.id.buttonReceiveBandwidthTest:
                if (receiveBandwidthTestAsync == null) {
                    receiveBandwidthTestAsync = new ReceiveBandwidthTestAsync()
                        .executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
                    Utilities.showToast("Started ReceiveBandwidthTestAsync");
                } else {
                    receiveBandwidthTestAsync.cancel(true);
                    receiveBandwidthTestAsync = null;
                    Utilities.showToast("Stopped ReceiveBandwidthTestAsync");
                }

                break;
        }
    }
}
