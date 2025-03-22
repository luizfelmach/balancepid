#include <Arduino.h>
#include <ESP32Servo.h>
#include <InfluxDbClient.h>
#include <WiFi.h>

#define SENSOR 34
#define SERVO 26

const char* SSID             = "Home";
const char* PASSWORD         = "2aw6d13z";
const char* INFLUXDB_HOST    = "http://10.0.0.177:8086";
const char* INFLUXDB_DB_NAME = "pid";

float kp       = 8;
float ki       = 0.2;
float kd       = 3000;
float setpoint = 25;

float error = 0, error_prev = 0;
float pid_p, pid_i, pid_d, pid_total;
float distance;
float servo_angle;

unsigned long elapsed, period = 50;

Servo          servo;
InfluxDBClient client(INFLUXDB_HOST, INFLUXDB_DB_NAME);
Point          sensor("measurement");

bool  connect_to_wifi();
bool  connect_to_influxdb();
void  send_to_influxdb();
float read_distance(int samples);

void setup() {
    Serial.begin(115200);
    delay(1000);

    if (!connect_to_wifi()) {
        for (;;);
    }
    if (!connect_to_influxdb()) {
        for (;;);
    }

    pinMode(SENSOR, INPUT);
    servo.attach(SERVO);
    elapsed = millis();
}

void loop() {
    if (millis() > elapsed + period) {
        elapsed  = millis();
        distance = read_distance(100);
        error    = setpoint - distance;

        pid_p = kp * error;
        pid_i = pid_i + (ki * error);
        pid_d = kd * ((error - error_prev) / (float)period);

        pid_i = pid_i > 300 ? 300 : pid_i;
        pid_i = pid_i < -300 ? -300 : pid_i;

        pid_total   = pid_p + pid_i + pid_d;
        servo_angle = map(pid_total, -150, 150, 180, 90);

        servo_angle = servo_angle > 180 ? 180 : servo_angle;
        servo_angle = servo_angle < 90 ? 90 : servo_angle;

        servo.write(servo_angle);
        error_prev = error;
        send_to_influxdb();
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
    sensor.addField("angle", (float)map(servo_angle, 90, 180, -45, 45));

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
    return 30;
    return 27.728 * pow(adc, -1.2045);
}

bool connect_to_wifi() {
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
    return WiFi.status() == WL_CONNECTED;
}

bool connect_to_influxdb() {
    client.setConnectionParamsV1(INFLUXDB_HOST, INFLUXDB_DB_NAME, "", "");
    return client.validateConnection();
}