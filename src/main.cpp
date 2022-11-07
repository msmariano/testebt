#include <Arduino.h>
#include "BluetoothSerial.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ArduinoJson.h"
#include <SPIFFS.h>
#include <FS.h>
#include <WiFi.h>
#include "WiFiClient.h"

#define _30S 30*1000

BluetoothSerial SerialBT;
DynamicJsonDocument config(4096);
DynamicJsonDocument doc(4096);
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
      Serial.println("Arquivo Cfg:"+sConfig);
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

void taskConfig(void *arg) {
  Serial.println("Iniciando taskConfig");
  while(true){
    if(SerialBT.available())
    {
      String data = SerialBT.readStringUntil('\r');
      Serial.println(data);
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
    vTaskDelay(1000);
  }
   vTaskDelete(NULL);
}

void WiFiEvent(WiFiEvent_t event)
{
  switch(event){
    case SYSTEM_EVENT_STA_CONNECTED:
      WiFi.enableIpV6();     
    break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("Conectado...");
      Serial.println(WiFi.localIP().toString());
      config["statusConWifi"] = "Conectado com IP "+ WiFi.localIP().toString();
      break;
    case SYSTEM_EVENT_GOT_IP6:
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconectado...");
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
  WiFi.mode(WIFI_AP_STA);
  //WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE,INADDR_NONE);
  WiFi.setAutoConnect(true);
  WiFi.setHostname("Neuverse");
  WiFi.setAutoReconnect(true);
  WiFi.softAP("neuverse", "neuverse");
  WiFi.onEvent(WiFiEvent);
  config["statusConWifi"] = "Iniciado";
  if(ssid != "" && pass != "") {
    
    config["statusConWifi"] = "Conectando com "+ssid+" "+pass;
    Serial.println("Conectado Wifi "+ssid+" "+pass);
    WiFi.begin(ssid.c_str(), pass.c_str());
     xTaskCreatePinnedToCore(taskCliente, "taskCliente",2048*4,NULL,4,NULL,APP_CPU_NUM);

  }
}


String atualizar(int pin,String status)
{
  DynamicJsonDocument doc(4096);
  //int id = config["conectorSessao"]["id"];
  String nome = config["conectorSessao"]["nome"];
  String usuario = config["conectorSessao"]["usuario"];
  String senha = config["conectorSessao"]["senha"];
  String btnsJson  = config["buttonIOTSessao"];
  String retorno = "";

  DynamicJsonDocument buttonIOTs(4096);
  deserializeJson(buttonIOTs,btnsJson);

  for(int i=0;i<buttonIOTs.size();i++) {
    int out = buttonIOTs[i]["gpioNumControle"];
    if(out == pin){
      buttonIOTs[i]["status"] = status;
      break;
    }
  }  
  doc["status"] = "NOTIFICACAO";
  doc["tipo"] = "IOT";
  doc["nome"] = nome;
  doc["usuario"] = usuario;
  doc["senha"]   = senha;
  doc["iot"]["name"] = "Testando....";
  doc["iot"]["jSon"] = btnsJson;
  doc["buttons"] = buttonIOTs;
  serializeJson(doc, retorno);
  return retorno + "\r\n";
}

String login()
{
  DynamicJsonDocument doc(4096);
  //int id = config["conectorSessao"]["id"];
  String nome = config["conectorSessao"]["nome"];
  String usuario = config["conectorSessao"]["usuario"];
  String senha = config["conectorSessao"]["senha"];
  String btnsJson  = config["buttonIOTSessao"];
  String retorno = "";

  DynamicJsonDocument buttonIOTs(4096);
  deserializeJson(buttonIOTs,btnsJson);



  
  doc["status"] = "LOGIN";
  doc["tipo"] = "IOT";
  doc["nome"] = nome;
  doc["usuario"] = usuario;
  doc["senha"]   = senha;
  doc["iot"]["name"] = "Testando....";
  doc["iot"]["jSon"] = btnsJson;
  doc["buttons"] = buttonIOTs;
  serializeJson(doc, retorno);
  return retorno + "\r\n";
}

String alive(String id)
{
  DynamicJsonDocument doc(2048); 
  String retorno = "";
  doc["id"] = id;
  doc["usuario"] = config["conectorSessao"]["usuario"];;
  doc["nome"] = config["conectorSessao"]["nome"];
  doc["tipo"] = "CONTROLEREMOTO";
  doc["senha"]   =  config["conectorSessao"]["senha"];
  doc["iot"]["id"] = 0;
  doc["iot"]["name"] = "";
  doc["status"] = "ALIVE";
  serializeJson(doc, retorno);
  return retorno + "\r\n";
}


void taskAlive(void *arg) {
  par * p  = (par*)arg;
  String id = p->par1;
  DynamicJsonDocument conector(2048);
  while(cli.connected()) {
    timeoutAlive = 1;
    cli.print(alive(id));
    vTaskDelay(_30S);
    if(timeoutAlive == 1){
      cli.stop();
    }
  }
  vTaskDelete(NULL);
}

void processar(String dados) {
  DynamicJsonDocument conector(4096);
  //Serial.println("Dados:"+dados);
  deserializeJson(conector,dados);
  String idConector = conector["id"];
  String stConector = conector["status"];
  if(stConector == "LOGIN_OK"){
    Serial.println("LOGIN_OK");
    par * p = new par;
    p->par1 = idConector;
    xTaskCreatePinnedToCore(taskAlive, "taskAlive",2048*4,(void*)p,4,NULL,APP_CPU_NUM);
  }
  else if(stConector == "CONECTADO"){
    Serial.println("CONECTADO");
    timeoutAlive = 0;
  }
  else if(stConector == "PROCESSARBTN"){    
      Serial.println("PROCESSARBTN");
     for(int i=0;i<conector["buttons"].size();i++) {
        if(conector["buttons"][i]["status"] == "OFF") {
          String gpioNum = conector["buttons"][i]["gpioNumControle"];
          Serial.println("OFF:"+gpioNum);
          digitalWrite( conector["buttons"][i]["gpioNumControle"],LOW);
        }
        else{
          String gpioNum = conector["buttons"][i]["gpioNumControle"];
          Serial.println("ON:"+gpioNum);
          digitalWrite( conector["buttons"][i]["gpioNumControle"],HIGH);
        }
     }
  }
}

void atualizarServidor(void *p) {
    pino * pin = (pino *)p;
  
    switch(digitalRead(pin->pinOUT)){
      case HIGH:
       if(cli.connected())
          Serial.println(atualizar(pin->pinOUT,"HIGH"));
        break;
      case LOW:
         if(cli.connected())
          Serial.println(atualizar(pin->pinOUT,"LOW"));
        break;
    }

     vTaskDelete(NULL);

}

void trataInterrupcao(void * p){
  pino * pin = (pino *)p;
  Serial.println("Interrupcao do pino "+String(pin->pinInterrupcao));
  if(pin->direcao == INPUT){
    if(pin->tipo == "0"){
      for(int i=0; i<10 ;i++){
        if(pin->pinosOUT[i]!=-1){
          switch(digitalRead(pin->pinIN)){
            case HIGH:
              //Serial.println("Ativando:"+String(pin->pinosOUT[i]));
              digitalWrite(pin->pinosOUT[i],HIGH);
              break;
            case LOW:
              //Serial.println("Desativando:"+String(pin->pinosOUT[i]));
              digitalWrite(pin->pinosOUT[i],LOW);
              break;
          }
        }
      }
    }
  }
  else if(pin->direcao == OUTPUT){
    xTaskCreatePinnedToCore(atualizarServidor, "atualizarServidor",8192,pin,4,NULL,APP_CPU_NUM);

  } 
}

void inicializarButtons(String jSon){
  DynamicJsonDocument buttonIOTs(4096);
  deserializeJson(buttonIOTs,jSon);
  
  int pinosIn[10][10];
  for(int i =0; i <10 ; i++)
    for(int j =0; j <10 ; j++)
      pinosIn[i][j] = -1;
  

  for(int i=0;i<buttonIOTs.size();i++) {

    
    String status = buttonIOTs[i]["status"];
    String funcao = buttonIOTs[i]["funcao"];
    int in = buttonIOTs[i]["gpioNum"];
    int out = buttonIOTs[i]["gpioNumControle"];

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
          if(buttonIOTs[i]["funcao"] == "HOLD" || buttonIOTs[i]["funcao"] == "KEY")
            pinosIn[j][1] = 0;
          else if(buttonIOTs[i]["funcao"] == "PUSH")
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
      Serial.println("Configurando pino in " +String(pinosIn[i][0])+ " para pinos out ");
      for(int l =0;l<10;l++){
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




void taskCliente(void *arg) {
  
  int32_t timeout = 10000;
  while(1) {
   if (cli.connect(server.c_str(), porta,timeout)) {
      cli.print(login());
      while (cli.connected())
      {
        if (cli.available())
        {
          String dados = cli.readStringUntil('\n');
          if (dados == "")
            continue;
          processar(dados);
        }
      }
      cli.stop();   
    }
    else
      vTaskDelay(1000);
  }
  vTaskDelete(NULL);
}



void setup() {
  Serial.begin(115200);  
  delay(5000);
  Serial.println("Inciando.");
  SerialBT.begin("neuverseBTIot");
   xTaskCreatePinnedToCore(taskConfig, "taskConfig",2048,NULL,4,NULL,APP_CPU_NUM);
  if(leArquivoConfig() != ""){
    String btnsJson  = config["buttonIOTSessao"];
    inicializarButtons(btnsJson);
    config["statusConWifi"] = "Aguardando";
    String serverLocal =config["servidorSessao"]["endereco"];
    server = serverLocal;
    porta = config["servidorSessao"]["porta"];
    String ssidLocal = config["ssidSessao"]["ssid"];
    String passLocal = config["ssidSessao"]["password"];
    ssid = ssidLocal;
    pass = passLocal;
    iniciaWifi();   
  }
  else
    Serial.println("Configuracao vazia");
} 

void loop() {
}