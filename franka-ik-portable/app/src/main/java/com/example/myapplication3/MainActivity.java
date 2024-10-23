package com.example.myapplication3;

import androidx.appcompat.app.AppCompatActivity;

import android.location.Address;
import android.os.Bundle;
import android.view.View;
import android.widget.EditText;
import android.widget.TextView;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class MainActivity extends AppCompatActivity {

    static double x_temp=1,y_temp=1,z_temp=1;
    private double[] Joints_rad=new double[]{0,0,0,0,0,0,0};
    TextView receive_x,receive_y,receive_z;
    private TextView[] receive_joints;
    private EditText IP_input;
    private String IP_destination;

    static {
        System.loadLibrary("native-lib");
        System.loadLibrary("test1");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        receive_joints=new TextView[7];
        receive_joints[0]=findViewById(R.id.receive_joint1);
        receive_joints[1]=findViewById(R.id.receive_joint2);
        receive_joints[2]=findViewById(R.id.receive_joint3);
        receive_joints[3]=findViewById(R.id.receive_joint4);
        receive_joints[4]=findViewById(R.id.receive_joint5);
        receive_joints[5]=findViewById(R.id.receive_joint6);
        receive_joints[6]=findViewById(R.id.receive_joint7);
        for(int count=0;count<7;count++) receive_joints[count].setText("0000.000");
        receive_x=findViewById(R.id.receive_x);
        receive_x.setText("0000.000");
        receive_y=findViewById(R.id.receive_y);
        receive_y.setText("0000.000");
        receive_z=findViewById(R.id.receive_z);
        receive_z.setText("0000.000");

    }

    public void StartThread(View view) {
        IP_input=findViewById(R.id.IP_input);
        IP_destination=IP_input.getText().toString();
        UDPServerThread Receive=new UDPServerThread();
        Receive.start();
    }

    public class UDPServerThread extends Thread{
        @Override
        public void run(){
            while (true) {
                if (true) {
                    try {
                        DatagramSocket socket=new DatagramSocket(6600);
                        byte[] data =new byte[1024];
                        DatagramPacket packet=new DatagramPacket(data, data.length);
                        socket.receive(packet);
                        String MessageReceived=new String(data, "UTF-8");
                        MessageReceived=MessageReceived.trim();
                        String x_display=MessageReceived.substring(MessageReceived.indexOf("x")+1,MessageReceived.indexOf("y"));
                        String y_display=MessageReceived.substring(MessageReceived.indexOf("y")+1,MessageReceived.indexOf("z"));
                        String z_display=MessageReceived.substring(MessageReceived.indexOf("z")+1);
                        double x_uncoded = Double.parseDouble(x_display);
                        double y_uncoded = Double.parseDouble(y_display);
                        double z_uncoded = Double.parseDouble(z_display);
                        runOnUiThread(() -> {
                            receive_x.setText(x_display);
                            receive_y.setText(y_display);
                            receive_z.setText(z_display);
                        });
                        Joints_rad[0]=IK_joint1(x_uncoded, y_uncoded, z_uncoded);
                        Joints_rad[1]=IK_joint2(x_uncoded, y_uncoded, z_uncoded);
                        Joints_rad[2]=IK_joint3(x_uncoded, y_uncoded, z_uncoded);
                        Joints_rad[3]=IK_joint4(x_uncoded, y_uncoded, z_uncoded);
                        //Joints_rad[3]=-0.070;
                        Joints_rad[4]=IK_joint5(x_uncoded, y_uncoded, z_uncoded);
                        Joints_rad[5]=IK_joint6(x_uncoded, y_uncoded, z_uncoded);
                        Joints_rad[6]=IK_joint7(x_uncoded, y_uncoded, z_uncoded);
                        runOnUiThread(() -> {
                            for(int count=0;count<7;count++){
                                receive_joints[count].setText(String.valueOf(Joints_rad[count]));
                            }
                        });
                        String IKSolvedResult=SynthesizeData();
                        InetAddress address=packet.getAddress();
                        int port_send=6601;
                        byte[] SendData=IKSolvedResult.getBytes("UTF-8");
                        DatagramPacket packet2_send=new DatagramPacket(SendData, SendData.length, address, port_send);
                        DatagramPacket packet3_send=new DatagramPacket(SendData, SendData.length, InetAddress.getByName(IP_destination), 52001);
                        socket.send(packet2_send);
                        socket.send(packet3_send);
                        socket.close();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    }

    //public native String stringFromJNI();
    public native double IK_joint1(double x,double y,double z);
    public native double IK_joint2(double x,double y,double z);
    public native double IK_joint3(double x,double y,double z);
    public native double IK_joint4(double x,double y,double z);
    public native double IK_joint5(double x,double y,double z);
    public native double IK_joint6(double x,double y,double z);
    public native double IK_joint7(double x,double y,double z);

    private String SynthesizeData(){
        String JointInfo_return="";
        String JointInfo_temp = "",Data_temp;
        for(int count=0;count<7;count++){
            Data_temp=String.valueOf(Joints_rad[count]);
            if(!Data_temp.contains(".")){
                if(Data_temp.startsWith("-")){
                    if(Data_temp.length()>=4) JointInfo_temp="-"+Data_temp.substring(Data_temp.length()-3)+".000";
                    else {
                        JointInfo_temp=Data_temp.substring(1);
                        while (JointInfo_temp.length()<3){
                            JointInfo_temp="0"+JointInfo_temp;
                        }
                        JointInfo_temp="-"+JointInfo_temp+".000";
                    }
                    JointInfo_return=JointInfo_return+JointInfo_temp;
                }
                else {
                    if(Data_temp.length()>=4) JointInfo_temp=Data_temp.substring(Data_temp.length()-4)+".000";
                    else {
                        JointInfo_temp=Data_temp;
                        while(JointInfo_temp.length()<4){
                            JointInfo_temp="0"+JointInfo_temp;
                        }
                        JointInfo_temp=JointInfo_temp+".000";
                    }
                    JointInfo_return=JointInfo_return+JointInfo_temp;
                }
            }
            else{
                if(Data_temp.startsWith("-")){
                    int Index=Data_temp.indexOf('.');
                    if(Index>=4&&(Data_temp.length()-Index)>=4) JointInfo_temp="-"+Data_temp.substring(Index-3,Index)+Data_temp.substring(Index,Index+4);
                    else if (Index>=4&&(Data_temp.length()-Index)<4){
                        JointInfo_temp="-"+Data_temp.substring(Index-3);
                        while ((JointInfo_temp.length()-Index)<4){
                            JointInfo_temp=JointInfo_temp+"0";
                            Index=JointInfo_temp.indexOf('.');
                        }
                    }
                    else if (Index<4&&(Data_temp.length()-Index)>=4){
                        JointInfo_temp=Data_temp.substring(1,Index+4);
                        while (Index<3){
                            JointInfo_temp="0"+JointInfo_temp;
                            Index=JointInfo_temp.indexOf('.');
                        }
                        Index=Data_temp.indexOf('.');
                        JointInfo_temp="-"+JointInfo_temp;
                    }
                    else {
                        JointInfo_temp=Data_temp.substring(1);
                        while ((JointInfo_temp.length()-Index)<4){
                            JointInfo_temp=JointInfo_temp+"0";
                            Index=JointInfo_temp.indexOf('.');
                        }
                        while (Index<3){
                            JointInfo_temp="0"+JointInfo_temp;
                            Index=JointInfo_temp.indexOf('.');
                        }
                        Index=Data_temp.indexOf('.');
                        JointInfo_temp="-"+JointInfo_temp;
                    }
                    JointInfo_return=JointInfo_return+JointInfo_temp;
                }
                else {
                    int Index=Data_temp.indexOf('.');
                    if(Index>=4&&(Data_temp.length()-Index)>=4) JointInfo_temp=Data_temp.substring(Index-4,Index)+Data_temp.substring(Index,Index+4);
                    else if(Index>=4&&(Data_temp.length()-Index)<4){
                        JointInfo_temp=Data_temp.substring(Index-4);
                        while ((JointInfo_temp.length()-Index)<4){
                            JointInfo_temp=JointInfo_temp+"0";
                            Index=JointInfo_temp.indexOf('.');
                        }
                    }
                    else if(Index<4&&(Data_temp.length()-Index)>=4){
                        JointInfo_temp=Data_temp.substring(0,Index+4);
                        while (Index<4){
                            JointInfo_temp="0"+JointInfo_temp;
                            Index=JointInfo_temp.indexOf('.');
                        }
                        Index=Data_temp.indexOf('.');
                    }
                    else {
                        JointInfo_temp=Data_temp;
                        while ((JointInfo_temp.length()-Index)<4){
                            JointInfo_temp=JointInfo_temp+"0";
                            Index=JointInfo_temp.indexOf('.');
                        }
                        while (Index<4){
                            JointInfo_temp="0"+JointInfo_temp;
                            Index=JointInfo_temp.indexOf('.');
                        }
                        Index=Data_temp.indexOf('.');
                    }
                    JointInfo_return=JointInfo_return+JointInfo_temp;
                }
            }
        }
        JointInfo_return=JointInfo_return.trim();
        JointInfo_return=JointInfo_return+"0000000000000000000000000000000000000000";
        return JointInfo_return;
    }
}