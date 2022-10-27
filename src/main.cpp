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
DynamicJsonDocument config(2048);
DynamicJsonDocument doc(2048);
WiFiClient cli;
int timeoutAlive = 0;
typedef struct par {
  String par1;
}par;


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
        deserializeJson(config, sConfig);
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
      String data = SerialBT.readString();
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
}

void WiFiEvent(WiFiEvent_t event)
{
  switch(event){
    case SYSTEM_EVENT_STA_CONNECTED:
      WiFi.enableIpV6();
    break;
    case SYSTEM_EVENT_STA_GOT_IP:
      break;
    case SYSTEM_EVENT_GOT_IP6:
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
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
  if(config["ssidSessao"]["ssid"] != "" && config["ssidSessao"]["password"] != "") {
    String ssid = config["ssidSessao"]["ssid"];
    String pass = config["ssidSessao"]["password"];
    WiFi.begin(ssid.c_str(), pass.c_str());
  }
}

String login()
{
  DynamicJsonDocument doc(2048);
  //int id = config["conectorSessao"]["id"];
  String nome = config["conectorSessao"]["nome"];
  String usuario = config["conectorSessao"]["usuario"];
  String senha = config["conectorSessao"]["senha"];
  String retorno = "";
  //doc["id"] = id;
  doc["nome"] = nome;
  doc["usuario"] = usuario;
  doc["senha"]   = senha;
  //doc["iot"]["id"] = 0;
  doc["iot"]["name"] = "nameIot";
  doc["status"] = "LOGIN";
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
}

void processar(String dados) {
  DynamicJsonDocument conector(2048);
  deserializeJson(conector,dados);
  String idConector = conector["id"];
  String stConector = conector["status"];
  if(stConector == "LOGIN_OK"){
    par * p = new par;
    p->par1 = idConector;
    xTaskCreatePinnedToCore(taskAlive, "taskAlive",2048*4,(void*)p,4,NULL,APP_CPU_NUM);
  }
  else if(stConector == "CONECTADO"){
    timeoutAlive = 0;
  }
}

void taskCliente(void *arg) {
  
  String server = config["servidorSessao"]["endereco"];
  uint16_t porta = config["servidorSessao"]["porta"];
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
}

void setup() {
  Serial.begin(115200);  
  SerialBT.begin("neuverseBTIot");
  leArquivoConfig();
  xTaskCreatePinnedToCore(taskConfig, "taskConfig",2048,NULL,4,NULL,APP_CPU_NUM);
  xTaskCreatePinnedToCore(taskCliente, "taskCliente",2048*4,NULL,4,NULL,APP_CPU_NUM);
}

void loop() {
}