package com.example.frankacontrolclient;

import android.os.Bundle;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.View;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class MainActivity extends AppCompatActivity {

    public String IPAddress;
    public int Port;
    private EditText IPAddress_text;
    private EditText Port_text;
    private EditText[] Joints_rad;
    private TextView[] Joints_degree;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Joints_rad=new EditText[7];
        Joints_degree=new TextView[7];
        IPAddress_text=findViewById(R.id.IP_type);
        Port_text=findViewById(R.id.port_type);
        Joints_rad[0]=findViewById(R.id.Joint1_rad);
        Joints_degree[0]=findViewById(R.id.Joint1_degree);
        Joints_rad[1]=findViewById(R.id.Joint2_rad);
        Joints_degree[1]=findViewById(R.id.Joint2_degree);
        Joints_rad[2]=findViewById(R.id.Joint3_rad);
        Joints_degree[2]=findViewById(R.id.Joint3_degree);
        Joints_rad[3]=findViewById(R.id.Joint4_rad);
        Joints_degree[3]=findViewById(R.id.Joint4_degree);
        Joints_rad[4]=findViewById(R.id.Joint5_rad);
        Joints_degree[4]=findViewById(R.id.Joint5_degree);
        Joints_rad[5]=findViewById(R.id.Joint6_rad);
        Joints_degree[5]=findViewById(R.id.Joint6_degree);
        Joints_rad[6]=findViewById(R.id.Joint7_rad);
        Joints_degree[6]=findViewById(R.id.Joint7_degree);
        for(int count=0;count<7;count++){
            int finalCount = count;
            Joints_rad[count].addTextChangedListener(new TextWatcher() {
                @Override
                public void beforeTextChanged(CharSequence s, int start, int count, int after) {            }
                @Override
                public void onTextChanged(CharSequence s, int start, int before, int count) {            }
                @Override
                public void afterTextChanged(Editable s) {
                    Joints_rad[finalCount].removeTextChangedListener(this);
                    Joints_degree[finalCount].setText(ConvertToDegree(Joints_rad[finalCount]));
                    Joints_rad[finalCount].addTextChangedListener(this);
                }
            });
        }
    }

    public void ConfigureAndSendData(View view) throws IOException {
        SetParameters();
        if(DataIsLegal()) {
            String jointInfo_final = SynthesizeData();
            SendMessage(IPAddress, Port, jointInfo_final);
            toastMessage("Sent successfully!");
        }
    }

    private void SetParameters(){
        IPAddress=IPAddress_text.getText().toString();
        Port=Integer.parseInt(Port_text.getText().toString());
    }

    private String SynthesizeData(){
        String JointInfo_return="";
        String JointInfo_temp = "",Data_temp;
        for(int count=0;count<7;count++){
            Data_temp=Joints_rad[count].getText().toString();
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

    public boolean DataIsLegal(){
        int[] Illegal=new int[8];
        Illegal[7]=0;
        for(int count=0;count<7;count++){
            String Info_temp=Joints_degree[count].getText().toString();
            if(Info_temp.equals("Invalid")||Info_temp.equals("Enter Value")||Info_temp.equals("Degree1")||Info_temp.equals("Degree2")||Info_temp.equals("Degree3")
                 ||Info_temp.equals("Degree4")||Info_temp.equals("Degree5")||Info_temp.equals("Degree6")||Info_temp.equals("Degree7")) {
                Illegal[count]=1;
                Illegal[7]+=1;
            }
            else Illegal[count]=0;
        }
        if(Illegal[7]==0) return true;
        else {
            String IllegalInfo=" ";
            String be,FinalInfo,indicate;
            for(int count=0;count<7;count++){
                if(Illegal[count]==1) IllegalInfo=IllegalInfo+JointName(count)+", ";
            }
            IllegalInfo=IllegalInfo.substring(1,IllegalInfo.length()-2);
            if(Illegal[7]==1) {
                be=" is";
                indicate="it";
            }
            else {
                be=" are";
                indicate="them";
            }
            FinalInfo=IllegalInfo+be+" illegal. \nPlease check "+indicate+" again.";
            toastMessage(FinalInfo);
            return false;
        }
    }

    private String ConvertToDegree(EditText rad){
        String textContent_temp=rad.getText().toString();
        if(textContent_temp.isEmpty()) return "Enter Value";
        else if(isNumeric(textContent_temp)) {
            float rad_temp = Float.parseFloat(textContent_temp);
            float degree_temp = (float) (rad_temp * 180 / Math.PI);
            return Float.toString(degree_temp);
        }
        else return "Invalid";
    }

    /*
    private String ConvertToRadius(EditText deg){
        float deg_temp=Float.parseFloat(deg.getText().toString());
        float rad_temp= (float) (deg_temp*Math.PI/180);
        return Float.toString(rad_temp);
    }
    */

    public void SendMessage(String IP, int port, String Message) throws IOException {
        int port_temp=port;
        InetAddress address= InetAddress.getByName(IP);
        byte[] SendData=Message.getBytes("UTF-8");
        DatagramPacket packet=new DatagramPacket(SendData,SendData.length,address, port_temp);
        DatagramSocket socket=new DatagramSocket();
        socket.send(packet);
        //socket.close();
    }

    public void toastMessage(String msg) {
        Toast.makeText(this, msg, Toast.LENGTH_SHORT).show();
    }

    public static boolean isNumeric(String Target) {
        if (Target == null || Target.length() == 0) return false;
        try {
            Integer.parseInt(Target);
            return true;
        } catch (NumberFormatException e1) {
            try {
                Double.parseDouble(Target);
                return true;
            } catch (NumberFormatException e2) {
                try {
                    Float.parseFloat(Target);
                    return true;
                } catch (NumberFormatException e3) {
                    return false;
                }
            }
        }
    }

    public String JointName(int number){
        switch (number){
            case 0: return "Joint1";
            case 1: return "Joint2";
            case 2: return "Joint3";
            case 3: return "Joint4";
            case 4: return "Joint5";
            case 5: return "Joint6";
            case 6: return "Joint7";
            default: return null;
        }
    }
}