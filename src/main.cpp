#include <Arduino.h>
#include "BluetoothSerial.h"
#include "ArduinoJson.h"
#include <SPIFFS.h>
#include <FS.h>
#include <WiFi.h>
#include "WiFiClient.h"

#define _30S 30*1000

//Lista de devices do IOT
DynamicJsonDocument buttons(4096);
DynamicJsonDocument conector(4096);
DynamicJsonDocument config(4096);
DynamicJsonDocument conectorProcessar(4096);
DynamicJsonDocument doc(4096);

BluetoothSerial SerialBT;


bool interrompido = false;
bool inicialzado = false;
bool taskAliveIniciado = false;
WiFiClient cli;
int timeoutAlive = 0;
typedef struct par {
  String par1;
}par;
typedef struct pino {
  int pinInterrupcao;
  int pinOUT;
  int pinIN;
  String tipo;
  int direcao;
  int pinosOUT[10] ={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
}pino;
String server;
uint16_t porta; 
String ssid;
String pass;
void taskCliente(void *arg);

void gravaArquivoConfigRest(String content) {
  if (SPIFFS.begin(true))
  {
    SPIFFS.remove(F("/config.json"));
    File fConfig = SPIFFS.open(F("/config.json"), "w");
    if (fConfig) {
      fConfig.print(content);
      fConfig.close();
    }
    SPIFFS.end();
  }
}

String leArquivoConfig() {
  String sConfig = "";
  if(SPIFFS.begin(true)) {
    File fConfig = SPIFFS.open(F("/config.json"), "r");
    if (fConfig) {
      sConfig = fConfig.readString();
      if (sConfig != "" && sConfig != NULL) {
        DeserializationError error = deserializeJson(config, sConfig);
        if (error != NULL) {
          return "";
        }
      }
      else
        sConfig ="";
      fConfig.close();
    }    
  }
  return sConfig;
}


void trataInterrupcao(void * p){
  pino * pin = (pino *)p;
  if(pin->direcao == INPUT){
    Serial.println("Interrupcao de entrada do pino "+String(pin->pinInterrupcao));
    if(pin->tipo == "0"){
      for(int i=0; i<10 ;i++){
        if(pin->pinosOUT[i]!=-1){
          switch(digitalRead(pin->pinIN)){
            case HIGH:
              digitalWrite(pin->pinosOUT[i],HIGH);
              break;
            case LOW:
              digitalWrite(pin->pinosOUT[i],LOW);
              break;
          }
        }
      }
    }
  }
  else if(pin->direcao == OUTPUT) {
    interrompido = true;
  } 
}

void inicializarButtons(String jSon){
  deserializeJson(buttons,jSon);  
  int pinosIn[10][10];
  for(int i =0; i <10 ; i++)
    for(int j =0; j <10 ; j++)
      pinosIn[i][j] = -1;
  for(int i=0;i<buttons.size();i++) {    
    String status = buttons[i]["status"];
    String funcao = buttons[i]["funcao"];
    int in = buttons[i]["gpioNum"];
    int out = buttons[i]["gpioNumControle"];
    Serial.println("Incializando IN["+String(in)+"] OUT["+String(out)+"]");
    pinMode(in,INPUT_PULLUP);
    pinMode(out,OUTPUT);
    if(status == "ON")
      digitalWrite(out,HIGH);
    else
      digitalWrite(out,LOW);       
    pino * pinOut = new pino;
    pinOut->pinOUT = out;
    pinOut->pinIN = in;
    pinOut->tipo = funcao;
    pinOut->direcao = OUTPUT;
    pinOut->pinInterrupcao = out;
    attachInterruptArg(out, trataInterrupcao,pinOut, CHANGE);    
    bool bAchou = false;
    for(int j =0; j <10 ; j++){
      if(pinosIn[j][0] == in){
        for(int k =2; k <10 ; k++){
          if(pinosIn[j][k] == -1){
            pinosIn[j][k] = out;  
            break;
          }
        }
        bAchou = true;
        break;
      }
    }
    if(!bAchou){
      for(int j =0; j <10 ; j++){
        if(pinosIn[j][0] == -1){
          if(buttons[i]["funcao"] == "HOLD" || buttons[i]["funcao"] == "KEY")
            pinosIn[j][1] = 0;
          else if(buttons[i]["funcao"] == "PUSH")
            pinosIn[j][1] = 1;
          pinosIn[j][0] = in;
          pinosIn[j][2] = out;
          break;
        }
      }  
    }    
  }  
  for(int i =0; i <10 ; i++)
  {
    if(pinosIn[i][0] != -1){
      pino * pinIN = new pino;
      int k =0;
      for(int j =2 ; j<10;j++){        
        if(pinosIn[i][j] != -1)
          pinIN->pinosOUT[k++] = pinosIn[i][j]; 
      }
      Serial.print("Configurando pino in " +String(pinosIn[i][0])+ " para pinos out ");
      for(int l =0;l<10;l++){
        if(pinIN->pinosOUT[l]!=-1)
          Serial.print(String(pinIN->pinosOUT[l])+" " );
      }
      Serial.println(".");
      pinIN->pinOUT =  -1;
      pinIN->pinIN = pinosIn[i][0];
      pinIN->tipo =  String(pinosIn[i][1]);
      pinIN->direcao = INPUT;
      pinIN->pinInterrupcao = pinosIn[i][0];
      if(String(pinosIn[i][1]) == "0")
        attachInterruptArg(pinosIn[i][0], trataInterrupcao,pinIN, CHANGE);
      else if(String(pinosIn[i][1]) == "1")
        attachInterruptArg(pinosIn[i][0], trataInterrupcao,pinIN, FALLING);
    }
  }
}

void leBlueConfig(){
  if(SerialBT.available())
    {
      String data = SerialBT.readStringUntil('\r');
      DeserializationError error = deserializeJson(doc, data);
      if (error == NULL) {
        if(doc["acao"] == "retConfig") {
          config["acao"] = "retConfig";
          config["resultado"] = "Ok";
          String jSon="";
          serializeJson(config, jSon); 
          SerialBT.println(jSon);
        }
        else if(doc["acao"] == "setConfig") {
          deserializeJson(config, data);  
          config["resultado"] = "Ok/restart"; 
          String jSon="";
          serializeJson(config, jSon);      
          gravaArquivoConfigRest(jSon);    
          SerialBT.println(jSon);      
        }
      }  
    }

}

void WiFiEvent(WiFiEvent_t event)
{
  switch(event){
    case SYSTEM_EVENT_STA_CONNECTED:
      //WiFi.enableIpV6();     
    break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("Ip do IOT "+ WiFi.localIP().toString());
      break;
    case SYSTEM_EVENT_GOT_IP6:
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconectado do WiFi");
      break;
    case SYSTEM_EVENT_STA_LOST_IP:
      break;
    case SYSTEM_EVENT_STA_WPS_ER_PBC_OVERLAP:
    case SYSTEM_EVENT_WIFI_READY:
    case SYSTEM_EVENT_SCAN_DONE:
    case SYSTEM_EVENT_STA_START:
    case SYSTEM_EVENT_STA_STOP:
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
    case SYSTEM_EVENT_AP_START:
    case SYSTEM_EVENT_AP_STOP:
    case SYSTEM_EVENT_AP_STACONNECTED:
    case SYSTEM_EVENT_AP_STADISCONNECTED:
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
    case SYSTEM_EVENT_ETH_START:
    case SYSTEM_EVENT_ETH_STOP:
    case SYSTEM_EVENT_ETH_CONNECTED:
    case SYSTEM_EVENT_ETH_DISCONNECTED:
    case SYSTEM_EVENT_ETH_GOT_IP:
    case SYSTEM_EVENT_MAX:
      break;
  }
}

void iniciaWifi() {
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.onEvent(WiFiEvent);
  config["statusConWifi"] = "Iniciado";
  if(ssid != "" && pass != "") {
    config["statusConWifi"] = "Conectando com "+ssid+" "+pass;
    Serial.println("Conectado Wifi "+ssid+" "+pass);
    WiFi.begin(ssid.c_str(), pass.c_str());
    Serial.print("Conectando WiFi.");
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("WiFi conectado.");
  }
}

String prConSt(String st){
  String retorno;
  conector["status"] = st;
  serializeJson(conector, retorno);
  return retorno;
}

void processar(String dados) {  
  deserializeJson(conectorProcessar,dados);
  conector["idConector"] = conectorProcessar["idConector"];
  conector["status"] =  conectorProcessar["status"];
  String status = conector["status"];
  String idConector = conector["idConector"];
  Serial.println("Processar: id"+idConector+ " "+status);
  if(status == "LOGIN_OK"){
    cli.println(prConSt("ALIVE"));
  }
  else if(status == "CONECTADO"){
    //timeoutAlive = 0;
    cli.println(prConSt("ALIVE"));
  }
  else if(status == "PROCESSARBTN"){   
    for(int i=0;i<conectorProcessar["buttons"].size();i++) {
      if(conectorProcessar["buttons"][i]["funcao"] == "KEY") {
        if(conectorProcessar["buttons"][i]["status"] == "OFF") {
          digitalWrite( conectorProcessar["buttons"][i]["gpioNumControle"],LOW);
        }
        else if(conectorProcessar["buttons"][i]["status"] == "ON") {
          digitalWrite( conectorProcessar["buttons"][i]["gpioNumControle"],HIGH);
        }
      }
      else if(conectorProcessar["buttons"][i]["funcao"] == "PUSH") {

      }  
    }   
  }
}

void setup() {
  Serial.begin(115200);  
  delay(5000);
  Serial.println("");
  Serial.println("Incializando.");
  SerialBT.begin("neuverseBTIot");
  if(leArquivoConfig() != ""){
    String btnsJson  = config["buttonIOTSessao"];
    inicializarButtons(btnsJson);
    config["statusConWifi"] = "Aguardando";
    String serverLocal =config["servidorSessao"]["endereco"];
    server = serverLocal;
    porta = config["servidorSessao"]["porta"];
    String ssidLocal = config["ssidSessao"]["ssid"];
    String passLocal = config["ssidSessao"]["password"];
    String nome = config["conectorSessao"]["nome"];
    String usuario = config["conectorSessao"]["usuario"];
    String senha = config["conectorSessao"]["senha"];
    //Inicializando conector do IOT
    String jSonBTN;
    serializeJson(buttons,jSonBTN);
    conector["nome"] = "neuverseBTIot";
    conector["status"] = "LOGIN";
    conector["nome"] = nome;
    conector["usuario"] = usuario;
    conector["senha"]   = senha;
    conector["iot"]["name"] = "Esp32IOTNeuverse";
    conector["iot"]["jSon"] = jSonBTN;
    conector["iot"]["tipoIOT"] = "IOT";
    conector["buttons"] = buttons;
    conector["tipo"] = "CONTROLEREMOTO";
    ssid = ssidLocal;
    pass = passLocal;
    iniciaWifi();   
    inicialzado = true;
    Serial.println("Inicializado");  
  }
  else
    Serial.println("Configuracao vazia");
 
} 

void loop() {  
   if (cli.connect(server.c_str(), porta,10000)) {
      cli.println(prConSt("LOGIN"));
      while (cli.connected())
      {
        if (cli.available())
        {
          String dados = cli.readStringUntil('\n');
          if (dados == "")
            continue;
          processar(dados);
        }
        if(interrompido){
          Serial.println("tratando interrupcao!");
          for(int i=0;i<conector["buttons"].size();i++){
            int out = digitalRead(buttons[i]["gpioNumControle"]);
            switch(out){
              case LOW:
                conector["buttons"][i]["status"] = "OFF";
                break;
              case HIGH:
                conector["buttons"][i]["status"] = "ON";  
                break;
            }
          }
          cli.println(prConSt("NOTIFICACAO"));
          interrompido = false;
        }
        leBlueConfig();
      }
      Serial.println("Desconectando do ServidorIOT.");   
    }
    leBlueConfig();
    if(timeoutAlive>500){
      timeoutAlive = 0;
      Serial.println("Timeout!");
    }
    else{
      timeoutAlive++;
    }
}

