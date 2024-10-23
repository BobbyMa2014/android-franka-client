package com.example.labas;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.RadioButton;
import android.widget.RadioGroup;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class MainActivity extends AppCompatActivity {

    private String TAG = "MainActivity";

    public static String ipString;
    //发送目的地的端口号，可在setting里随时修改
    public static int portNumber;

    public String p1Code = "8";
    public String p2Code = "8";
    public String p3Code = "8";
    public String p4Code = "8";
    public String TCode;
    public RadioGroup pGroup1,pGroup2,pGroup3,pGroup4;

    public static final String PRE_IP = "IP";
    public static final String PRE_PORT = "PORT";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        pGroup1 = (RadioGroup) findViewById(R.id.PGroup1);
        pGroup2 = (RadioGroup) findViewById(R.id.PGroup2);
        pGroup3 = (RadioGroup) findViewById(R.id.PGroup3);
        pGroup4 = (RadioGroup) findViewById(R.id.PGroup4);
        Button submitButton = findViewById(R.id.name_Submit);

        pGroup1.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                switch (checkedId){
                    case R.id.P11_id:
                        p1Code="1";
                        break;
                    case R.id.P12_id:
                        p1Code="0";
                        break;
                    default:
                        p1Code="8";
                        break;

                }
            }
        });

        pGroup2.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                switch (checkedId){
                    case R.id.P21_id:
                        p2Code="2";
                        break;
                    case R.id.P22_id:
                        p2Code="0";
                        break;
                    default:
                        p2Code="8";
                        break;
                }
            }
        });

        pGroup3.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                switch (checkedId){
                    case R.id.P31_id:
                        p3Code="3";
                        break;
                    case R.id.P32_id:
                        p3Code="0";
                        break;
                    default:
                        p3Code="8";
                        break;
                }
            }
        });

        pGroup4.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                switch (checkedId){
                    case R.id.P41_id:
                        p4Code="4";
                        break;
                    case R.id.P42_id:
                        p4Code="0";
                        break;
                    default:
                        p4Code="8";
                        break;
                }
            }
        });

        submitButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
            TCode = p1Code + p2Code + p3Code + p4Code;
            Log.i(TAG,"TCode: "+TCode);

            new Thread() {
                public void run() {
                    //将发送消息的函数写在线程中去，因为Android不允许在子线程中进行UI操作。
                    sendMsg(TCode);
                }
            }.start();
            }
        });
        getPRE(this);
    }

    //发送UDP数据
    public void sendMsg(String msg){
        Log.i("client send","...");
        //获取反馈的text组件
        try {
            //构造数据报套接字并将其绑定到本地主机上的任何可用端口。这里用的是0号，其他接口没试过。
            DatagramSocket socket = new DatagramSocket(0);

            //目标地址，这里是我的主机IP。但是现在只能在同一wifi下才能正常工作。

            InetAddress host = InetAddress.getByName(ipString);//192.168.0.113//192.168.3.3//192.168.3.2 dom

            //指定包要发送的目的地
            byte[] data=msg.getBytes();//这里指定UTF-8可以得到中文收发。其实不用指定也行，因为“activity_main”里的定义决定了程序是默认使用UTF-8编码。
            DatagramPacket request =new DatagramPacket(data,data.length, host, portNumber);

            //发送
            socket.send(request);
            Log.i("client send","success");

            /*
             * 接收服务器端响应的数据
             */
            /*
            //1.创建数据报，用于接收服务器端响应的数据
            byte[] data2=new byte[1024];
            DatagramPacket packet2=new DatagramPacket(data2, data2.length);
            //2.接收服务器响应的数据，程序会阻塞在这里，直到有消息被接受。
            socket.receive(packet2);
            //3.读取数据
            String reply=new String(data2, 0, packet2.getLength());
            //显示在feedback上
            feedback.setText(reply);

             */

        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            Log.i("Error", "...");
        }
    }

    public void getPRE(Context context){
        SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(context);
        ipString = pref.getString(PRE_IP,"192.168.3.2");
        portNumber = pref.getInt(PRE_PORT,8800);
        Log.i(TAG,"getPRE");
    }

    public void goSetting(View v){
        Intent myIntent = new Intent( this, Setting.class );
        this.startActivity( myIntent );
        overridePendingTransition(R.anim.enter_from_right,R.anim.leave_to_left);
    }

    public void ClearAll(View view) {
        pGroup1.clearCheck();
        pGroup2.clearCheck();
        pGroup3.clearCheck();
        pGroup4.clearCheck();
        p1Code = "8";
        p2Code = "8";
        p3Code = "8";
        p4Code = "8";
    }
}