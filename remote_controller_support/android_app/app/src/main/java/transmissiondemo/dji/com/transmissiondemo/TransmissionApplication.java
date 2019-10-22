package transmissiondemo.dji.com.transmissiondemo;

import android.app.Application;
import android.content.Context;
import com.secneo.sdk.Helper;

/**
 * Created by sickness on 19.2.2018.
 */

public class TransmissionApplication extends Application {

    @Override
    protected void attachBaseContext(Context paramContext) {
        super.attachBaseContext(paramContext);

        // Initializes DJI SDK classes
        Helper.install(TransmissionApplication.this);
    }

}
