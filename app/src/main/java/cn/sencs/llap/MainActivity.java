package cn.sencs.llap;

import android.Manifest;
import android.content.pm.PackageManager;
import android.os.Handler;
import android.os.Message;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;

import java.util.ArrayList;

public class MainActivity extends AppCompatActivity {
    private SeekBar sb;
    private TextView tv;
    private int distance;

    private Handler handler=new Handler(){
        @Override
        public void handleMessage(Message msg) {
            if(msg.what==0){
                tv.setText(Integer.toString(distance)+" mm");
                sb.setProgress(distance);
            }
        }
    };

    static {
        System.loadLibrary("LLAP");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        initPermission();
        setContentView(R.layout.activity_main);

        tv = (TextView) findViewById(R.id.textView);
        sb = (SeekBar)findViewById(R.id.seekBar);
        final Button bt = (Button)findViewById(R.id.button);
        tv.setText("0 mm");

        bt.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Begin();
            }
        });
    }

    public void refresh(int progress){
        Message msg=new Message();
        distance = progress;
        msg.what=0;
        handler.sendMessage(msg);
    }

    private void initPermission() {
        String permissions[] = {Manifest.permission.RECORD_AUDIO,
                Manifest.permission.ACCESS_NETWORK_STATE,
                Manifest.permission.INTERNET,
                Manifest.permission.READ_PHONE_STATE,
                Manifest.permission.WRITE_EXTERNAL_STORAGE
        };

        ArrayList<String> toApplyList = new ArrayList<String>();

        for (String perm :permissions){
            if (PackageManager.PERMISSION_GRANTED != ContextCompat.checkSelfPermission(this, perm)) {
                toApplyList.add(perm);
                //进入到这里代表没有权限.

            }
        }
        String tmpList[] = new String[toApplyList.size()];
        if (!toApplyList.isEmpty()){
            ActivityCompat.requestPermissions(this, toApplyList.toArray(tmpList), 123);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {

    }


    public native void Begin();
}
