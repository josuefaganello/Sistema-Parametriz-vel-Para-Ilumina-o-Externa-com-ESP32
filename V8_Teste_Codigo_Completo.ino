// Projeto De TCC
// Acadêmico Josué Kaleb Faganello Donadel
// SISTEMA PARAMETRIZÁVEL PARA ILUMINAÇÃO EXTERNA BASEADA NA PLATAFORMA ESP32
// Professor Orientador: Prof. Dr. Alexandre Roque
// Versão 9.0

#include <WiFi.h> // Biblioteca WiFi
#include <WiFiClientSecure.h> // Biblioteca WiFi Seguro
#include <MQTT.h> // Biblioteca MQTT
#include <WebServer.h>// Biblioteca p/ portal html
#include <Preferences.h> // Biblioteca p/ salvar perfil
#include "time.h" // Biblioteca p/ horário
#include <ZMPT101B.h> // Bilioteca sensor de tensão
#include <ACS712.h> // Bilioteca sensor de corrente
#include <SPI.h> // Bilioteca comunicação SPI
#include <SD.h> // Bilioteca cartão SD
#include <FS.h> // Bilioteca "File System"

// ----------------- CREDENCIAIS WiFi -----------------
const char* ssid = "URISANb";
const char* password = "urisan99";

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -10800; // UTC-3
const int daylightOffset_sec = 0;

WebServer server(80);
Preferences prefs;

//-------------------CREDENCIAIS LOSANT MQTT------------------
const char* MQTT_CLIENT_ID = "68f64632d72bcc6c8173f3f3"; // "DEVICE ID"
const char* MQTT_USERNAME = "51730969-a6f7-4d59-b20f-9791efeca73e"; //"ACCESS KEY"
const char* MQTT_PASSWORD = "d112bbd1ae6a4e2994cbb38658c5d4759d0b23ae856f40104dc2e4f82d0259bd"; //"ACCESS SECRET"

WiFiClientSecure wifiClient; // Cliente seguro para TLS
MQTTClient mqttClient(256); // Buffer de 256 bytes

// ----------------- PINAGEM -----------------
// Relés (8 canais)
int relayPins[8] = {13, 14, 27, 26, 25, 33, 32, 17};

//---------------SENSOR DE LUMINOSIDADE---------------
const int sensorLumPin = 21;  // digital input (LOW = escuro)

// ----------------- DEFINIÇÕES ZMPT101b -----------------
ZMPT101B voltageSensor(35, 60.0); // freq 60.0 (ajuste se 50Hz)
#define ZMPT_SENSITIVITY 879.0f // Valor alto devido a alimentação em 3.3V (contém certa imprecisão)

// ----------------- DEFINIÇÕES ACS712 -----------------
ACS712  ACS(34, 3.3, 4095, 185); // (Pino, Vref, Resolução ADC, mV/A)
int midPoint = 2698; // Ponto médio obtido via código de testes
int NoisemV = 35; // Ruído aproximado em miliVolts

// ----------------- TEMPO DE ESCURIDÃO -----------------
#define TEMPO_ESCURO_MS (1 * 60 * 1000UL) // 1 min para teste

// ----------------- INTERVALOS -----------------
unsigned long lastMeasurementMillis = 0;
const unsigned long MEAS_INTERVAL_MS = 5000UL; // 5s medições

// ----------------- ESTRUTURA PARA PERFIL -----------------
struct Perfil {
  String sequencia;      // "1,0,1,0,..." ou "1010101"
  bool usarLuminosidade; // Usar ou não a luminosidade p/ ligar
  bool horarioAtivo;     // ativar controle por horário
  String horaLigado;     // "HH:MM"
  String horaDesligado;  // "HH:MM"
  // Novos campos para Subhorário
  String subMask;        // "10101000" - '1' = participa do subhorario (desliga na horaSubDesligar)
  String horaSubDesligar;// "HH:MM" - hora única em que os marcados em subMask serão desligados
};
Perfil perfil;

// ---------- SALVAR PERFIL NA MEMÓRIA ----------
void salvarPerfil(const Perfil &perfil) {
  prefs.begin("perfil", false);  // Inicia namespace "perfil"
  prefs.putBool("usarLuminosidade", perfil.usarLuminosidade);
  prefs.putString("sequencia", perfil.sequencia);
  prefs.putBool("horarioAtivo", perfil.horarioAtivo);
  prefs.putString("horaLigado", perfil.horaLigado);
  prefs.putString("horaDesligado", perfil.horaDesligado);
  // Novos
  prefs.putString("subMask", perfil.subMask);
  prefs.putString("horaSubDesligar", perfil.horaSubDesligar);
  prefs.end();
  Serial.println("Perfil salvo na memória!");
}

// ---------- CARREGAR PERFIL DA MEMÓRIA ----------
void carregarPerfil(Perfil &perfil) {
  prefs.begin("perfil", true);  // Modo somente leitura
  perfil.usarLuminosidade = prefs.getBool("usarLuminosidade", true); // Padrão = true
  perfil.sequencia = prefs.getString("sequencia", "11111111");   // Valor padrão se não houver nada salvo (8 ligados)
  perfil.horarioAtivo = prefs.getBool("horarioAtivo", false);
  perfil.horaLigado = prefs.getString("horaLigado", "00:00");
  perfil.horaDesligado = prefs.getString("horaDesligado", "00:00");
  // Novos
  perfil.subMask = prefs.getString("subMask", "00000000"); // Por padrão nenhum participa do subhorario
  perfil.horaSubDesligar = prefs.getString("horaSubDesligar", "00:00");
  prefs.end();
  Serial.println("Perfil carregado da memória!");
}

// ----------------- ESTADO ESCURIDÃO -----------------
unsigned long darkStartMillis = 0;
bool darkTriggered = false;

// ----------------- MEDIÇÕES -----------------
float ultimaTensaoRMS = 0.0f;
float ultimaCorrenteRMS = 0.0f;
float Potencia = 0.0f;

// ----------------- MÉDIA 5 MIN -----------------
float somaPotencia5min = 0.0f;
int contMedicoes5min = 0;
float Potencia5min = 0.0f;
unsigned long ultimoReset5min = 0;

// ----------------- RECONEXÃO MQTT (não-bloqueante, backoff) -----------------
unsigned long lastMqttAttempt = 0;
unsigned long mqttRetryInterval = 5000UL; // inicialmente 5s
const unsigned long MQTT_MAX_RETRY_INTERVAL = 60000UL; // cap 1min

// ----------------- FUNÇÕES AUXILIARES (PARA WEB SERVER VIA ESP32) -----------------

// Aplica sequência no formato "1,0,1,0,..." ou "1010..." (RETORNA vetor de 8 valores)
void parseSequenciaToVals(const String &seq, int vals[8]) {
  for(int i=0;i<8;i++) vals[i] = 0;
  String s = seq; s.trim();
  if (s.length() == 0) return;
  if (s.indexOf(',') >= 0) {
    int start = 0; int r = 0;
    while(r < 8 && start < s.length()) {
      int comma = s.indexOf(',', start);
      String token = (comma == -1) ? s.substring(start) : s.substring(start, comma);
      token.trim();
      vals[r++] = token.toInt() ? 1 : 0;
      if(comma == -1) break;
      start = comma + 1;
    }
  } else {
    for(int r=0;r<8;r++) vals[r] = (r<s.length() && s.charAt(r)=='1') ? 1 : 0;
  }
}

// Converte "HH:MM" para minutos desde meia-noite, retorna -1 em caso de formato inválido
int horaStringParaMinutos(const String &hhmm) {
  if(hhmm.length()<4) return -1;
  int hh = hhmm.substring(0,2).toInt();
  int mm = hhmm.substring(3,5).toInt();
  if(hh<0||hh>23||mm<0||mm>59) return -1;
  return hh*60+mm;
}

// Verifica se horaAtualMin está no intervalo [inicio, fim) com suporte a wrap-around
bool estaNoIntervalo(const String &inicio, const String &fim, int horaAtualMin) {
  int ini = horaStringParaMinutos(inicio);
  int fimMin = horaStringParaMinutos(fim);
  if(ini<0||fimMin<0||horaAtualMin<0) return false;
  if(ini<=fimMin) return (horaAtualMin>=ini && horaAtualMin<fimMin);
  else return (horaAtualMin>=ini || horaAtualMin<fimMin);
}

// ----------------- GERAR/PROCURAR ARQUIVO .CSV -----------------
#define ARQUIVO_CSV "/medicoes.csv"

void salvarCSV(float tensao, float corrente, float potencia) {
  // Cria arquivo com cabeçalho se não existir
  if(!SD.exists(ARQUIVO_CSV)) {
    File file = SD.open(ARQUIVO_CSV, FILE_WRITE);
    if(file) {
      file.println("Data;Hora;Tensao;Corrente;Potencia");
      file.close();
    }
  }

  // Abre arquivo para adicionar linha
  File file = SD.open(ARQUIVO_CSV, FILE_APPEND);
  if(file){
    struct tm timeinfo;
    getLocalTime(&timeinfo);

    char bufData[11], bufHora[9];
    strftime(bufData, 11, "%d/%m/%Y", &timeinfo);
    strftime(bufHora, 9, "%H:%M:%S", &timeinfo);

    file.print(bufData); file.print(";");
    file.print(bufHora); file.print(";");
    file.print(tensao, 3); file.print(";");
    file.print(corrente / 1000.0, 3); file.print(";");
    file.print(potencia, 3);
    file.println();
    file.close();
  }
}

// ----------------- PORTAL WEB HTML -----------------

void handleRoot() {
  int vals[8] = {0,0,0,0,0,0,0,0};
  parseSequenciaToVals(perfil.sequencia, vals);
  String sub = perfil.subMask;
  while (sub.length() < 8) sub += "0";

  String html = R"rawliteral(
  <!DOCTYPE html>
  <html lang="pt-BR">
  <head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Configurar Iluminação</title>
    <style>
      body { font-family: Arial, sans-serif; background-color: #f2f4f8; margin: 0; padding: 20px; text-align: center; }
      h2 { font-size: 26px; font-weight: bold; color: #333; text-align: center; margin-bottom: 12px; }
      .top-row { max-width: 900px; margin: 0 auto 18px; display:flex; gap:12px; justify-content:center; align-items:center; flex-wrap:wrap; }
      .top-row .field { font-weight:bold; }
      .relay-container { display: grid; grid-template-columns: repeat(4, 1fr); gap: 18px; justify-items: center; margin: 12px auto 18px; max-width: 900px; }
      .relay { background: #fff; border-radius: 12px; padding: 12px; box-shadow: 0 2px 6px rgba(0,0,0,0.08); width: 150px; }
      .relay .label { font-weight:bold; color:#222; margin-bottom:8px; display:block; }
      .small-note { font-size:12px; color:#666; margin-top:6px; }
      .switch { position: relative; display: inline-block; width: 50px; height: 28px; }
      .switch input { opacity: 0; width: 0; height: 0; }
      .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; transition: .25s; border-radius: 28px; }
      .slider:before { position: absolute; content: ""; height: 20px; width: 20px; left: 4px; bottom: 4px; background-color: white; transition: .25s; border-radius: 50%; }
      input:checked + .slider { background-color: #4CAF50; }
      input:checked + .slider:before { transform: translateX(22px); }
      .sub-checkbox { margin-top:8px; display:block; font-size:13px; }
      .buttons { display:flex; justify-content:center; gap:14px; margin-top:12px; }
      button { background-color: #0078d7; border:none; color:white; padding:10px 16px; border-radius:10px; font-weight:bold; cursor:pointer; font-size:14px; }
      .download-btn { background-color:#28a745; }
      button:hover { transform: scale(1.03); }
      .time-input { padding:6px 8px; border-radius:6px; border:1px solid #ccc; }
      .section { background:#fff; padding:12px; border-radius:10px; max-width:900px; margin:10px auto; box-shadow:0 1px 4px rgba(0,0,0,0.06); }
      .row { display:flex; gap:12px; align-items:center; justify-content:center; flex-wrap:wrap; }
    </style>
  </head>
  <body>
    <h2>💡 Configurar Iluminação 💡</h2>
    <form action="/save" method="post">
      <div class="section top-row">
        <div class="field">
          <label><input type="checkbox" name="horario" )rawliteral";
  if (perfil.horarioAtivo) html += " checked";
  html += R"rawliteral(> <strong>Usar Horário</strong></label>
        </div>
        <div class="field">
          <label><input type="time" name="ligar" class="time-input" value=")rawliteral";
  html += perfil.horaLigado;
  html += R"rawliteral("> <span class="small-note">Hora ligar</span></label>
        </div>
        <div class="field">
          <label><input type="time" name="desligar" class="time-input" value=")rawliteral";
  html += perfil.horaDesligado;
  html += R"rawliteral("> <span class="small-note">Hora desligar</span></label>
        </div>
        <div class="field">
          <label><input type="checkbox" name="luminosidade" )rawliteral";
  if (perfil.usarLuminosidade) html += " checked";
  html += R"rawliteral(> <strong>Usar sensor de luminosidade</strong></label>
        </div>
      </div>

      <div class="section">
        <div style="text-align:left; font-weight:bold; margin-bottom:8px;">Sub-horário (marque as luminárias que devem desligar em conjunto)</div>
        <div class="row" style="justify-content:center; gap:8px; margin-bottom:8px;">
          <label style="font-size:13px;">Hora Sub-Desligar: <input type="time" name="subdesligar" class="time-input" value=")rawliteral";
  html += perfil.horaSubDesligar;
  html += R"rawliteral("></label>
        </div>
        <div class="small-note" style="text-align:left;">Marque abaixo quais luminárias participam do sub-horário (elas serão forçadas a desligar na hora configurada)</div>
      </div>

      <div class="relay-container">
  )rawliteral";

  for (int i = 0; i < 8; ++i) {
    String relayName = String("relay") + String(i); // relay0..relay7
    String subName = String("sub") + String(i);     // sub0..sub7
    html += "<div class='relay'>";
    html += "<span class='lamp-icon'>💡</span>";
    html += "<span class='label'>Luminária " + String(i+1) + "</span>";
    html += "<label class='switch'><input type='checkbox' name='" + relayName + "'";
    if (i < 8 && vals[i] == 1) html += " checked";
    html += "><span class='slider'></span></label>";
    char c = (i < sub.length()) ? sub.charAt(i) : '0';
    html += "<label class='sub-checkbox'><input type='checkbox' name='" + subName + "'";
    if (c == '1') html += " checked";
    html += "> Participa do sub-horário</label>";
    html += "</div>";
  }

  html += R"rawliteral(
      </div>

      <div class="buttons">
        <button type="submit">💾 Salvar Perfil 💾</button>
        <button type="button" class="download-btn" onclick="window.location.href='/download'">⬇️ Baixar CSV ⬇️ </button>
      </div>
    </form>
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

//-------------------------FUNÇÃO SALVAR PERFIL DE OPERAÇÃO -------------------------

void handleSave() {
  if (server.hasArg("seq")) {
    perfil.sequencia = server.arg("seq");
  } else {
    String newSeq = "";
    for (int i = 0; i < 8; ++i) {
      String rname = String("relay") + String(i);
      if (server.hasArg(rname)) newSeq += "1";
      else newSeq += "0";
    }
    perfil.sequencia = newSeq;
  }

  // 2) Luminosidade e Horário
  perfil.usarLuminosidade = server.hasArg("luminosidade");
  perfil.horarioAtivo = server.hasArg("horario");

  // 3) Horas ligar/desligar (se vierem no POST)
  if (server.hasArg("ligar")) perfil.horaLigado = server.arg("ligar");
  if (server.hasArg("desligar")) perfil.horaDesligado = server.arg("desligar");

  // 4) SubMask: monta a máscara sub0..sub7 (subhorário)
  String newSubMask = "";
  for (int i = 0; i < 8; ++i) {
    String sname = String("sub") + String(i);
    if (server.hasArg(sname)) newSubMask += "1";
    else newSubMask += "0";
  }
  perfil.subMask = newSubMask;

  // 5) Hora Sub-Desligar
  if (server.hasArg("subdesligar")) perfil.horaSubDesligar = server.arg("subdesligar");

  // 6) Persistir e redirecionar
  salvarPerfil(perfil);

  // redireciona de volta para '/'
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

//-----------------------BOTÃO PARA DOWNLOAD DO .CSV VIA HTML--------------------
void handleDownload() {
  if(SD.exists(ARQUIVO_CSV)){
    File file = SD.open(ARQUIVO_CSV);
    server.streamFile(file, "text/csv");
    file.close();
  } else {
    server.send(404,"text/plain","Arquivo não encontrado");
  }
}

//------------------------ CONEXÃO WIFI (com timeout) ------------------------

bool connectWiFiWithTimeout(unsigned long timeoutMs = 10000UL) {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.begin(ssid, password);
  Serial.print("Conectando ao Wi-Fi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    Serial.print(".");
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi conectado!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.macAddress());
    return true;
  } else {
    Serial.println("\nFalha ao conectar ao Wi-Fi (timeout).");
    return false;
  }
}

//------------------------ TENTAR CONEXÃO MQTT (COM TIMEOUT E BACKOFF) ------------------------

bool tryConnectMQTT(unsigned long timeoutMs = 8000UL) {
  if (mqttClient.connected()) return true;

  static bool mqttBeginCalled = false;
  if (!mqttBeginCalled) {
    // garante que setInsecure seja chamado primeiro
    wifiClient.setInsecure();
    mqttClient.begin("broker.losant.com", 8883, wifiClient);
    mqttBeginCalled = true;
  }

  Serial.print("Tentando conectar MQTT (Losant)...");
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("\nConectado ao Losant!");
      // reset retry interval on success
      mqttRetryInterval = 5000UL;
      lastMqttAttempt = millis();
      return true;
    }
    Serial.print(".");
    delay(200);
  }

  Serial.println("\nFalha ao conectar MQTT (timeout).");
  lastMqttAttempt = millis();
  mqttRetryInterval = min(mqttRetryInterval * 2, MQTT_MAX_RETRY_INTERVAL);
  return false;
}

//------------------------ FUNÇÃO connect() COORDENADORA (não-bloqueante) ------------------------

void connectCoordinator() {
  // tenta WiFi com timeout curto
  bool wifiOk = connectWiFiWithTimeout(10000UL);
  if (wifiOk) {
    // tenta MQTT, mas retorna rapidamente (não bloqueante indefinidamente)
    tryConnectMQTT(8000UL);
  } else {
    Serial.println("Pulando tentativa MQTT, sem Wi-Fi.");
  }
}

// ----------------- SETUP -----------------

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Iniciando sistema...");

  // Pinos relé iniciais
  for(int i=0;i<8;i++){ pinMode(relayPins[i],OUTPUT); digitalWrite(relayPins[i],LOW); }
  pinMode(sensorLumPin,INPUT);

  // Carrega perfil
  carregarPerfil(perfil);

  // Sensores
  voltageSensor.setSensitivity(ZMPT_SENSITIVITY);
  ACS.setMidPoint(midPoint); ACS.setNoisemV(NoisemV);

  // SD
if (!SD.begin(5)) {
  Serial.println("Falha no SD");
} else {
  Serial.println("SD Localizado");
}

  // NTP (não-bloqueante)
  configTime(gmtOffset_sec,daylightOffset_sec,ntpServer);

  // Webserver
  server.on("/",HTTP_GET,handleRoot);
  server.on("/save",HTTP_POST,handleSave);
  server.on("/download",HTTP_GET,handleDownload);
  server.begin();

  // Tenta conexão inicial (não-bloqueante)
  connectCoordinator();

  Serial.println("Setup concluído (serviços locais iniciados).");
}

// ----------------- LOOP -----------------
void loop() {
  // mqtt loop (processa mensagens rapidamente)
  mqttClient.loop();

  // Reconexão controlada (não-bloqueante, backoff)
  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastMqttAttempt >= mqttRetryInterval) {
      Serial.printf("MQTT desconectado. Tentando reconectar (intervalo %lu ms)\n", mqttRetryInterval);
      if (WiFi.status() == WL_CONNECTED) {
        tryConnectMQTT(8000UL);
      } else {
        // tenta reconectar WiFi rapidamente
        connectWiFiWithTimeout(5000UL);
      }
    }
  }

  unsigned long now = millis();
  if (now - ultimoReset5min >= 5UL * 60UL * 1000UL) { // 5 minutos (300000 ms)
    ultimoReset5min = now;

    // Calcula potência média no período
    if (contMedicoes5min > 0) {
      Potencia5min = somaPotencia5min / contMedicoes5min;
    } else {
      Potencia5min = 0;
    }

    // Reseta acumuladores
    somaPotencia5min = 0;
    contMedicoes5min = 0;

    // ----------- FORMATAÇÃO DO PAYLOAD EM JSON -----------
    String payload = "{\"data\": {\"Potencia5min\": ";
    payload += String(Potencia5min, 2);
    payload += "}}";

    // Caminho MQTT para o Losant (substitui automaticamente os IDs definidos)
    String topic = "losant/" + String(MQTT_CLIENT_ID) + "/state";

    // ----------- PUBLICAÇÃO NO LOSANT -----------

    if (mqttClient.connected()) {
      mqttClient.publish(topic.c_str(), payload.c_str());
      Serial.println("📡 Dados enviados ao Losant:");
      Serial.println(payload);
    } else {
      Serial.println("⚠️ Conexão MQTT não disponível — dados não enviados neste ciclo.");
      // Não chamar connectCoordinator() aqui para evitar loop bloqueante; reconexão é gerenciada acima
    }
  }

  // ATENÇÃO: servidor web deve ser atendido com frequência

  server.handleClient();

  if(millis() - lastMeasurementMillis >= MEAS_INTERVAL_MS){
    lastMeasurementMillis = millis();

    // Leitura
    ultimaTensaoRMS = voltageSensor.getRmsVoltage();
    ultimaCorrenteRMS = ACS.mA_AC();
    Potencia = ultimaTensaoRMS * (ultimaCorrenteRMS/1000.0);

    // Acumuladores para média
    somaPotencia5min += Potencia;
    contMedicoes5min++;

    // Salvar CSV
    salvarCSV(ultimaTensaoRMS, ultimaCorrenteRMS, Potencia);
  }

  // ----------- LÓGICA RELÉS + LUMINOSIDADE / HORÁRIO + SUBHORARIO -----------

  int estadoSensor = digitalRead(sensorLumPin); // LOW = escuro
  struct tm timeinfo;
  bool haveTime = getLocalTime(&timeinfo);
  int atualMin = haveTime ? timeinfo.tm_hour * 60 + timeinfo.tm_min : -1;

  bool aplicar = false;

  // --- Controle pelo modo selecionado ---
  if (perfil.usarLuminosidade) {

    // --- Controle por luminosidade ---
    if (estadoSensor == LOW) {
      if (darkStartMillis == 0)
        darkStartMillis = millis();
      else if (!darkTriggered && millis() - darkStartMillis >= TEMPO_ESCURO_MS) {
        darkTriggered = true;
        Serial.println("🌑 Escuridão detectada — ativando sequência");
      }
    } else {
      darkStartMillis = 0;
      darkTriggered = false;
    }

    aplicar = darkTriggered;
  } 
  else if (perfil.horarioAtivo && haveTime) {
    // --- Controle apenas por horário ---
    if (estaNoIntervalo(perfil.horaLigado, perfil.horaDesligado, atualMin))
      aplicar = true;
  }

  // --- Aplicação do estado dos relés ---
  if (aplicar) {
    int vals[8]; parseSequenciaToVals(perfil.sequencia, vals);

    // Verifica se o sub-horário já foi alcançado (aplica a todos os relés marcados)
    int subMin = horaStringParaMinutos(perfil.horaSubDesligar);
    bool subReached = false;
    if (subMin >= 0 && haveTime) {
      if (atualMin >= subMin) subReached = true;
    }

    // Aplica valores, exceto que relés em subMask serão desligados se subReached==true
    for (int i = 0; i < 8; i++) {
      char c = (i < perfil.subMask.length()) ? perfil.subMask.charAt(i) : '0';
      bool inSub = (c == '1');
      int desired = vals[i];
      if (inSub && subReached) desired = 0; // força desligado após horaSubDesligar
      digitalWrite(relayPins[i], desired ? HIGH : LOW);
    }
  } else {
    for (int i = 0; i < 8; i++) {
      digitalWrite(relayPins[i], LOW);
    }
  }

  delay(10); // mantém pequeno intervalo entre loops
}
