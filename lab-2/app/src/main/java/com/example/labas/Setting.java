package com.example.labas;

import android.content.Context;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.Gravity;
import android.view.View;
import android.widget.EditText;
import android.widget.RadioButton;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import static com.example.labas.MainActivity.PRE_IP;
import static com.example.labas.MainActivity.PRE_PORT;
import static com.example.labas.MainActivity.portNumber;
import static com.example.labas.MainActivity.ipString;


public class Setting extends AppCompatActivity {

    private EditText IP;
    private EditText Port;


    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.setting);

        IP = (EditText) findViewById(R.id.ipAddress);
        Port = (EditText) findViewById(R.id.portAddress);

        updateView();
    }

    public void save(View v){
        if(IP.getText().toString().isEmpty()){
            ipString = "192.168.0.113";
        }else{ipString = IP.getText().toString();}

        if(Port.getText().toString().isEmpty()){
            portNumber = 8001;
        }else{portNumber = Integer.parseInt(Port.getText().toString());}

        Log.i("MainActivity","portNumber:"+portNumber);

        setPRE(this);

        Toast toast = Toast.makeText(this, "Save is successful!", Toast.LENGTH_SHORT);
        toast.setGravity(Gravity.CENTER,0, 0);
        toast.show();

    }

    public void updateView(){
        if(!IP.getText().toString().equals("192.168.0.133")) IP.setText(ipString);
        //不能用if(!Port.getText().toString().equals("8800"))来判断
        if(Integer.parseInt(Port.getText().toString())==8800) Port.setText(String.valueOf(portNumber));
    }

    public void setPRE(Context context){
        SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(context);
        SharedPreferences.Editor editor = pref.edit();
        editor.putString(PRE_IP,ipString);
        editor.putInt(PRE_PORT,portNumber);
        editor.commit();
    }

    public void back( View v ) {
        this.finish();
        overridePendingTransition(R.anim.enter_from_left,R.anim.leave_to_right);
    }
}
