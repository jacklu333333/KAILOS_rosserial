
// #define SerialSpeed 57600
// #define SerialSpeed 115200
#define SerialSpeed 921600
// #define SerialSpeed 2000000

// #include <WiFiClientSecure.h>
#include <TinyGPSPlus.h>
#include "WiFi.h"
#if 0
const char* ssid     = "your-ssid";     // your network SSID (name of wifi network)
const char* password = "your-password"; // your network password
#else
const char *ssid = "John_S117";      // your network SSID (name of wifi network)
const char *password = "0988125378"; // your network password
#endif

#include <Wire.h>
// ROS
#include <ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <sensor_msgs/NavSatFix.h>

// I2C and Sensors

#include "SparkFun_MS5637_Arduino_Library.h"
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"
#include "MPU9250.h"
#include "eeprom_utils.h"

#define AL_ADDR 0x10
#define FRAME_ID "kailos_tag_link"

#define RXD2 16
#define TXD2 17

#define PWR_KEY 33
#define RST_KEY 25

TaskHandle_t Task1;
TaskHandle_t Task2;

// Blobal Sensor Valueables
SparkFun_Ambient_Light light(AL_ADDR);
MS5637 barometricSensor;
MPU9250 mpu;

// ROS
ros::NodeHandle nh;
// Baro
sensor_msgs::FluidPressure baro_press_msg;
sensor_msgs::Temperature baro_temp_msg;
ros::Publisher Baro_Press("/kailos_tag/baro/pressure", &baro_press_msg);
ros::Publisher Baro_Temp("/kailos_tag/baro/temperature", &baro_temp_msg);

// Imu
sensor_msgs::Imu imu_imu_msg;
ros::Publisher IMU_6DOF("/kailos_tag/imu", &imu_imu_msg);

// Mag
sensor_msgs::MagneticField imu_mag_msg;
ros::Publisher IMU_Mag("/kailos_tag/mag", &imu_mag_msg);

sensor_msgs::NavSatFix gps_msg;
ros::Publisher GPS_Pub("/kailos_tag/fix", &gps_msg);
bool pushGPS = false;
SemaphoreHandle_t xBinarySemaphore;
TinyGPSPlus gps;

void setup()
{
    // nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    // Serial.begin(SerialSpeed); // important !!! need this for reseting the baud rate
    Serial.println("Node Setup");
    Wire.begin();
    delay(2000);
    // wifiManger();

    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    pinMode(PWR_KEY, OUTPUT);    // sets the digital pin 13 as output
    digitalWrite(PWR_KEY, HIGH); // sets the digital pin 13 on
    delay(500);                  // waits for a second
    digitalWrite(PWR_KEY, LOW);  // sets the digital pin 13 off

    pinMode(RST_KEY, OUTPUT);    // sets the digital pin 13 as output
    digitalWrite(RST_KEY, HIGH); // sets the digital pin 13 on
    delay(500);                  // waits for a second
    digitalWrite(RST_KEY, LOW);  // sets the digital pin 13 off

    Serial.print("Initializing ");
    while (Serial2.available() == false)
    {
        Serial.print(".");
        delay(1000); // waits for a second
    }
    Serial.println("");
    Serial.print(Serial2.readString());
    Serial.println("------------------------------");
    Serial2.print("AT\r\n");
    while (Serial2.available() == false)
        ;
    Serial.println(Serial2.readString());
    Serial.println("Disable Echo");
    Serial2.print("ATE0\r\n");
    while (Serial2.available() == false)
        ;
    Serial.println(Serial2.readString());
    Serial.println("------------------------------");

    Serial.println("Trun on GPS");
    Serial2.print("AT+CGPS=1,1\r\n");
    while (Serial2.available() == false)
        ;
    Serial.println(Serial2.readString());
    Serial.println("------------------------------");

    Serial.println("Get First GPS INFO");
    Serial2.print("AT+CGPSINFO\r\n");
    while (Serial2.available() == false)
        ;
    Serial.println(Serial2.readString());
    Serial.println("------------------------------");

    Serial.println("Check NMEA mode");
    Serial2.print("AT+CGPSNMEA?\r\n");
    while (Serial2.available() == false)
        ;
    Serial.println(Serial2.readString());
    Serial.println("------------------------------");

    Serial.println("Set NMEA rate 10 hz");
    Serial2.print("AT+CGPSNMEARATE=1\r\n");
    while (Serial2.available() == false)
        ;
    Serial.println(Serial2.readString());
    Serial.println("------------------------------");

    Serial.println("Set NMEA Auto Report");
    Serial2.print("AT+CGPSINFOCFG=2,31\r\n");
    while (Serial2.available() == false)
        ;
    Serial.println(Serial2.readString());
    Serial.println("------------------------------");

    /****************************** IMU ******************************/
    // Connection Checking
    while (!mpu.setup(0x69))
    {
        Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
        delay(5000); // prevent jamming serial
    }

#if defined(ESP_PLATFORM) || defined(ESP8266)
    EEPROM.begin(0x80);
#endif

    delay(5000);

    // calibrate anytime you want to
    if (false) // WAIT_FOR_INPUT)
    {
        Serial.println("Accel Gyro calibration will start in 5sec.");
        Serial.println("Please leave the device still on the flat plane.");
        mpu.verbose(true);
        delay(5000);
        mpu.calibrateAccelGyro();

        Serial.println("Mag calibration will start in 5sec.");
        Serial.println("Please Wave device in a figure eight until done.");
        delay(5000);
        mpu.calibrateMag();

        print_calibration();
        mpu.verbose(false);

        // save to eeprom
        saveCalibration();
    }

    // load from eeprom for MPU
    loadCalibration();
    // set declination to Daejeon, Korea (36.374454498291016° N, 	127.36531066894531° E) 2022 Feb. 24
    mpu.setMagneticDeclination(8.52);

    Serial.println("Finished IMU Connection Setup");
    /****************************** Baro ******************************/
    // Connection Checking
    while (!barometricSensor.begin())
    {
        Serial.println("Barometer Connection is offline, please check your device");
        delay(5000); // prevent jamming serial
    }
    Serial.println("Finished Baro Connection Setup");
    /****************************** Light ******************************/
    // Connection Checking
    while (light.begin() == false)
    {
        Serial.println("Light sensor Connection is offline, please check your device");
        delay(5000); // prevent jamming serial
    }
    Serial.println("Finished Connection Setup");
    // ros

    // Baro
    nh.advertise(Baro_Press);
    nh.advertise(Baro_Temp);
    // Imu
    nh.advertise(IMU_6DOF);
    // Mag
    nh.advertise(IMU_Mag);
    // GPS
    nh.advertise(GPS_Pub);

    MSG_setup();
    Serial.println("Finished Setup");
    xBinarySemaphore = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(
        Task1code, /* Task function. */
        "Task1",   /* name of task. */
        10000,     /* Stack size of task */
        NULL,      /* parameter of the task */
        0,         /* priority of the task */
        &Task1,    /* Task handle to keep track of created task */
        0);        /* pin task to core 0 */
    delay(500);
    xTaskCreatePinnedToCore(
        Task2code, /* Task function. */
        "Task2",   /* name of task. */
        10000,     /* Stack size of task */
        NULL,      /* parameter of the task */
        2,         /* priority of the task */
        &Task2,    /* Task handle to keep track of created task */
        1);        /* pin task to core 1 */
    delay(500);
    Serial2.readString();
    xSemaphoreGive(xBinarySemaphore);
}
void loop()
{
}

void Task1code(void *pvParameters) // the High fps sensors (>10)
{
    while (true)
    {
        if (xSemaphoreTake(xBinarySemaphore, 10) == pdTRUE)
        {
            static unsigned long preTime = millis();
            int32_t result = 0;
            float pressure = 123.0, temperatureP = 456.0;
            float humidity, temperatureH;
            int32_t acc[3], gyr[3], mag[3];
            char nmeaStr[100];

            if (millis() - preTime > 100)
            {
                //        Serial.println(millis() - preTime);/
                preTime = millis();
                // Baro
                baro_temp_msg.temperature = barometricSensor.getTemperature();
                baro_press_msg.fluid_pressure = barometricSensor.getPressure() * 100;
                baro_temp_msg.header.stamp = baro_press_msg.header.stamp = nh.now();
                Baro_Press.publish(&baro_press_msg);

                Baro_Temp.publish(&baro_temp_msg);
                // Serial.println("Finished baro");
            }
            // IMU
            if (mpu.update())
            {
                imu_imu_msg.linear_acceleration.x = mpu.getAccX() * 9.80665;
                imu_imu_msg.linear_acceleration.y = mpu.getAccY() * 9.80665;
                imu_imu_msg.linear_acceleration.z = mpu.getAccZ() * 9.80665;

                imu_imu_msg.angular_velocity.x = mpu.getGyroX() * 0.01745329251;
                imu_imu_msg.angular_velocity.y = mpu.getGyroY() * 0.01745329251;
                imu_imu_msg.angular_velocity.z = mpu.getGyroZ() * 0.01745329251;

                imu_imu_msg.header.stamp = nh.now();
                IMU_6DOF.publish(&imu_imu_msg);

                // Serial.println("Finished imu_6dof");

                // MAG
                imu_mag_msg.magnetic_field.x = mpu.getMagY() * 0.0000001;
                imu_mag_msg.magnetic_field.y = mpu.getMagX() * 0.0000001;
                imu_mag_msg.magnetic_field.z = mpu.getMagZ() * -0.0000001;

                imu_mag_msg.header.stamp = imu_imu_msg.header.stamp;
                IMU_Mag.publish(&imu_mag_msg);
                // Serial.println("Finished mag");
            }
            if (pushGPS)
            {
                // Serial.println("here-----------------------");
                GPS_Pub.publish(&gps_msg);
                // Serial.println("here1-----------------------");
                pushGPS = false;
                // Serial.println("here2-----------------------");
            }

            // Serial.println("Task 1");
            xSemaphoreGive(xBinarySemaphore);
        }
        else
        {
            // Serial.println("Task 1 block");
        }
        vTaskDelay(3 / portTICK_PERIOD_MS);
        nh.spinOnce();
    }
}
void Task2code(void *pvParameters) // the Low fps sensors
{
    while (true)
    {
        while (Serial2.available() == false)
            ;

        String NMEA_Info = Serial2.readString();
        int start = 0;
        // Serial.println("here");
        // Serial.println(NMEA_Info);

        while (start <= NMEA_Info.lastIndexOf("\n"))
        {
            if (gps.encode(NMEA_Info.c_str()[start]))
                if (gps.location.isValid())
                    break;
            start++;
        }
        // Serial.println("--------------------------------------");
        if (gps.location.isValid())
        {
            if (xSemaphoreTake(xBinarySemaphore, 4) == pdTRUE)
            {
                gps_msg.header.stamp = nh.now();
                gps_msg.position_covariance_type = -1;
                // gps_msg.status.status=;
                // gps_msg.status.service=;
                gps_msg.latitude = gps.location.lat();
                gps_msg.longitude = gps.location.lng();
                gps_msg.altitude = gps.altitude.meters();

                // gps_msg.position_covariance = []; //[9]
                pushGPS = true;
                // Serial.println("Before");
                // GPS_Pub.publish(&gps_msg);
                // Serial.println("After");
                // Serial.println("Task 2 ");
                xSemaphoreGive(xBinarySemaphore);
            }
            else
            {
                // Serial.println("Task 2 block");
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void print_calibration()
{
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}

void wifiManger()
{

    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);

    // attempt to connect to Wifi network:
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        // wait 1 second for re-trying
        delay(1000);
    }

    Serial.print("Connected to ");
    Serial.println(ssid);
}

void MSG_setup()
{
    baro_press_msg.header.frame_id = FRAME_ID;
    baro_temp_msg.header.frame_id = FRAME_ID;

    imu_imu_msg.header.frame_id = FRAME_ID;
    imu_mag_msg.header.frame_id = FRAME_ID;

    gps_msg.header.frame_id = FRAME_ID;
}

void displayInfo()
{
    Serial.print(F("Location: "));
    if (gps.location.isValid())
    {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid())
    {
        if (gps.time.hour() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10)
            Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    }
    else
    {
        Serial.print(F("INVALID"));
    }

    Serial.println();
}
