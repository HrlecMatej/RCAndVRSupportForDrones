package transmissiondemo.dji.com.transmissiondemo.utilities;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

public class DjiThreadPool {

    private static DjiThreadPool instance;
    private final ThreadPoolExecutor threadPoolExec;
    private static int MAX_POOL_SIZE;
    private static final int KEEP_ALIVE = 30;
    BlockingQueue<Runnable> workQueue = new LinkedBlockingQueue<>();

    public static synchronized void post(final Runnable runnable) {
        if (instance == null) {
            instance = new DjiThreadPool();
        }
        instance.threadPoolExec.execute(runnable);
    }

    private DjiThreadPool() {
        final int coreNum = Runtime.getRuntime().availableProcessors();
        threadPoolExec = new ThreadPoolExecutor(
            coreNum,
            coreNum,
            KEEP_ALIVE,
            TimeUnit.SECONDS,
            workQueue);
    }

    public static void finish() {
        instance.threadPoolExec.shutdown();
    }

}
