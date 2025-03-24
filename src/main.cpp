#include <Arduino.h>
#include <ESP32Servo.h>
#include <InfluxDbClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <ezButton.h>
#include <rgb_lcd.h>

#define SENSOR 34
#define SERVO 19
#define DT 5
#define SW 4
#define CLK 18
#define BUTTON 25

#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

enum OperationMode {
    MODE_P,   // Atualizar apenas P
    MODE_I,   // Atualizar apenas I
    MODE_D,   // Atualizar apenas D
    MODE_PI,  // Atualizar P e I
    MODE_PD,  // Atualizar P e D
    MODE_ID,  // Atualizar I e D
    MODE_PID  // Atualizar todos
};

const char* SSID             = "S23";
const char* PASSWORD         = "12345678.";
const char* INFLUXDB_HOST    = "http://192.168.99.126:8086";
const char* INFLUXDB_DB_NAME = "pid";

float kp       = 8;
float ki       = 0.2;
float kd       = 3000;
float setpoint = 50;

float error = 0, error_prev = 0;
float pid_p, pid_i, pid_d, pid_total;
float distance;
float servo_angle;

int                 last_state_clk;
unsigned long       last_debounce_time = 0;
const unsigned long debounce_delay     = 10;

unsigned long elapsed, period               = 50;
unsigned long elapsed_influx, period_influx = 500;

OperationMode  currentMode = MODE_PID;
Servo          servo;
InfluxDBClient client(INFLUXDB_HOST, INFLUXDB_DB_NAME);
Point          sensor("measurement");
rgb_lcd        lcd;
ezButton       button(SW);
ezButton       toggle(BUTTON);

bool  connect_to_wifi();
bool  connect_to_influxdb();
void  send_to_influxdb();
float read_distance(int samples);
void  update_screen();
void  adjust_pid_values(float, float, float);
void  parallelTask(void* pvParameters);

void setup() {
    Serial.begin(115200);
    delay(1000);

    lcd.begin(16, 2);

    if (!connect_to_wifi()) {
        lcd.clear();
        lcd.print("Error connecting to WIFI");
        for (;;);
    }
    if (!connect_to_influxdb()) {
        for (;;);
    }

    pinMode(CLK, INPUT);
    pinMode(DT, INPUT);
    pinMode(SW, INPUT_PULLUP);
    pinMode(BUTTON, INPUT_PULLUP);
    button.setDebounceTime(50);
    toggle.setDebounceTime(50);
    pinMode(SENSOR, INPUT);
    servo.attach(SERVO);
    update_screen();
    last_state_clk = digitalRead(CLK);
    elapsed        = millis();
    elapsed_influx = millis();

    xTaskCreatePinnedToCore(parallelTask, "ParallelTask", 10000, NULL, 1, NULL,
                            1);
}

void loop() {
    button.loop();
    toggle.loop();
    int           state_clk    = digitalRead(CLK);
    int           state_dt     = digitalRead(DT);
    unsigned long current_time = millis();

    if (button.isPressed()) {
        currentMode =
            static_cast<OperationMode>((currentMode + 1) % (MODE_PID + 1));
        update_screen();
    }

    if (toggle.isPressed()) {
        if (setpoint == 50)
            setpoint = 25;
        else
            setpoint = 50;
    }

    if ((state_clk != last_state_clk) && (state_clk == LOW) &&
        (current_time - last_debounce_time > debounce_delay)) {
        last_debounce_time = current_time;

        if (state_dt != state_clk) {
            adjust_pid_values(0.5, 0.1, 50);
        } else {
            adjust_pid_values(-0.5, -0.1, -50);
        }
        update_screen();
    }

    last_state_clk = state_clk;

    if (millis() > elapsed + period) {
        elapsed  = millis();
        distance = read_distance(100);
        error    = setpoint - distance;

        pid_p = kp * error;
        pid_i = pid_i + (ki * error);
        pid_d = kd * ((error - error_prev) / (float)period);

        pid_i = pid_i > 300 ? 300 : pid_i;
        pid_i = pid_i < -300 ? -300 : pid_i;

        if (!(-25 < error && error < 25)) pid_i = 0;

        pid_total   = pid_p + pid_i + pid_d;
        servo_angle = map(pid_total, -150, 150, 180, 90);

        servo_angle = servo_angle > 180 ? 180 : servo_angle;
        servo_angle = servo_angle < 90 ? 90 : servo_angle;

        servo.write(servo_angle);
        error_prev = error;
    }
}

void send_to_influxdb() {
    sensor.clearFields();

    sensor.addField("distance", distance);
    sensor.addField("error", error);
    sensor.addField("setpoint", setpoint);
    sensor.addField("p", pid_p);
    sensor.addField("i", pid_i);
    sensor.addField("d", pid_d);
    sensor.addField("kp", kp);
    sensor.addField("ki", ki);
    sensor.addField("kd", kd);
    sensor.addField("angle", (float)map(servo_angle, 90, 180, -40, 40));

    if (!client.writePoint(sensor)) {
        Serial.print("Falha no envio de dados: ");
        Serial.println(client.getLastErrorMessage());
    }
}

float read_distance(int samples) {
    long sum = 0;
    for (int i = 0; i < samples; i++) sum = sum + analogRead(SENSOR);
    float adc_value = sum / samples;
    float adc       = adc_value * (3.3 / 4095.0);
    return 27.728 * pow(adc, -1.2045);
}

bool connect_to_wifi() {
    lcd.clear();
    lcd.setRGB(0, 0, 255);  // Azul - indica tentativa de conexão
    lcd.setCursor(0, 0);
    lcd.print("WiFi Connect...");
    lcd.setCursor(0, 1);
    lcd.print(SSID);

    WiFi.begin(SSID, PASSWORD);

    int dots    = 0;
    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED &&
           timeout < 30) {  // 15 segundos timeout
        delay(500);
        dots = (dots + 1) % 4;
        lcd.setCursor(strlen(SSID), 1);
        lcd.print("    ");  // Limpa os pontos anteriores
        lcd.setCursor(strlen(SSID), 1);
        for (int i = 0; i < dots; i++) {
            lcd.print(".");
        }
        timeout++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        lcd.clear();
        lcd.setRGB(0, 255, 0);  // Verde - indica sucesso
        lcd.setCursor(0, 0);
        lcd.print("WiFi Connected!");
        lcd.setCursor(0, 1);
        lcd.print("IP: ");
        lcd.print(WiFi.localIP().toString());
        delay(2000);  // Mostra a mensagem por 2 segundos
        return true;
    } else {
        lcd.clear();
        lcd.setRGB(255, 0, 0);  // Vermelho - indica falha
        lcd.setCursor(0, 0);
        lcd.print("WiFi Failed!");
        lcd.setCursor(0, 1);
        lcd.print("Check settings");
        delay(3000);  // Mostra a mensagem por 3 segundos
        return false;
    }
}

void parallelTask(void* pvParameters) {
    while (1) {
        send_to_influxdb();
        delay(100);
    }
}

bool connect_to_influxdb() {
    lcd.clear();
    lcd.setRGB(0, 0, 255);  // Azul - indica tentativa de conexão
    lcd.setCursor(0, 0);
    lcd.print("InfluxDB...");

    client.setConnectionParamsV1(INFLUXDB_HOST, INFLUXDB_DB_NAME, "", "");

    bool success = client.validateConnection();

    if (success) {
        lcd.setRGB(0, 255, 0);  // Verde - indica sucesso
        lcd.setCursor(0, 1);
        lcd.print("Connected!");
        delay(2000);  // Mostra a mensagem por 2 segundos
    } else {
        lcd.setRGB(255, 0, 0);  // Vermelho - indica falha
        lcd.setCursor(0, 1);
        lcd.print("Failed!");
        lcd.setCursor(0, 0);
        lcd.print("DB Error: ");
        lcd.setCursor(0, 1);
        // Tenta mostrar parte da mensagem de erro
        String errMsg = client.getLastErrorMessage();
        if (errMsg.length() > 16) {
            errMsg = errMsg.substring(0, 16);
        }
        lcd.print(errMsg);
        delay(3000);  // Mostra a mensagem por 3 segundos
    }

    return success;
}

void update_screen() {
    lcd.clear();

    struct RGBColor {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };

    static const RGBColor MODE_COLORS[] = {
        {255, 0, 0},     // RED - MODE_P
        {255, 255, 0},   // YELLOW - MODE_I
        {0, 255, 0},     // GREEN - MODE_D
        {0, 255, 255},   // TEAL - MODE_PI
        {0, 0, 255},     // BLUE - MODE_PD
        {255, 0, 255},   // VIOLET - MODE_ID
        {255, 255, 255}  // WHITE - MODE_PID
    };

    lcd.setRGB(MODE_COLORS[currentMode].r, MODE_COLORS[currentMode].g,
               MODE_COLORS[currentMode].b);

    lcd.setCursor(0, 0);
    lcd.print("P:");
    lcd.print(kp, 1);

    lcd.setCursor(8, 0);
    lcd.print("I:");
    lcd.print(ki, 1);

    lcd.setCursor(0, 1);
    lcd.print("D:");
    lcd.print(kd, 0);

    lcd.setCursor(8, 1);

    switch (currentMode) {
        case MODE_P:
            lcd.print("Mode:P");
            break;
        case MODE_I:
            lcd.print("Mode:I");
            break;
        case MODE_D:
            lcd.print("Mode:D");
            break;
        case MODE_PI:
            lcd.print("Mode:PI");
            break;
        case MODE_PD:
            lcd.print("Mode:PD");
            break;
        case MODE_ID:
            lcd.print("Mode:ID");
            break;
        case MODE_PID:
            lcd.print("Mode:PID");
            break;
    }
}

void adjust_pid_values(float p_delta, float i_delta, float d_delta) {
    switch (currentMode) {
        case MODE_P:
            kp += p_delta;
            break;
        case MODE_I:
            ki += i_delta;
            break;
        case MODE_D:
            kd += d_delta;
            break;
        case MODE_PI:
            kp += p_delta;
            ki += i_delta;
            break;
        case MODE_PD:
            kp += p_delta;
            kd += d_delta;
            break;
        case MODE_ID:
            ki += i_delta;
            kd += d_delta;
            break;
        case MODE_PID:
            kp += p_delta;
            ki += i_delta;
            kd += d_delta;
            break;
    }
}