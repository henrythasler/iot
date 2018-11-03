#include <Arduino.h>
#include "quaternionFilters.h"
#include "MPU9250.h"

#define SerialDebug true // Set to true to get Serial output for debugging

MPU9250 IMU(Wire, 0x68);
int status;

#define FILTER 20

float yaw[FILTER], pitch, roll;
float ax, ay, az, gx, gy, gz, mx, my, mz;
float a12, a22, a31, a32, a33;
float lin_ax, lin_ay, lin_az;
unsigned long Now, lastUpdate, transmit = 0;
float deltat;

int calibrated = true;
//float hxb=-14.0408, hxs=1.0277, hyb=63.3675, hys=0.9724, hzb=7.1777, hzs=1.0015;
float hxb=-11.9389, hxs=1.0558, hyb=64.6017, hys=0.9558, hzb=4.3337, hzs=0.9935;

unsigned int cnt = 0;

void setup()
{
    // serial to display data
    Serial.begin(115200);
    while (!Serial)
    {
        // Watchdog will reset µC eventually
    }

    // start communication with IMU
    status = IMU.begin();
    if (status < 0)
    {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while (1)
        {
            // Watchdog will reset µC
        }
    }

    // setting the accelerometer full scale range to +/-8G
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    // setting the gyroscope full scale range to +/-500 deg/s
    IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
    // setting DLPF bandwidth to 20 Hz
    //IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    // setting SRD to 19 for a 50 Hz update rate
    //IMU.setSrd(19);

    // set calibration values
    if (calibrated)
    {
        IMU.setMagCalX(hxb, hxs);
        IMU.setMagCalY(hyb, hys);
        IMU.setMagCalZ(hzb, hzs);
    }
}

void loop()
{
    if (!calibrated)
    {
        Serial.println("Starting calibration. Move the sensor slowly in every direction.");
        status = IMU.calibrateMag();
        Serial.println(status);
        Serial.println("Calibration finished");

        hxb = IMU.getMagBiasX_uT();
        hxs = IMU.getMagScaleFactorX();
        hyb = IMU.getMagBiasY_uT();
        hys = IMU.getMagScaleFactorY();
        hzb = IMU.getMagBiasZ_uT();
        hzs = IMU.getMagScaleFactorZ();

        Serial.print("hxb=");
        Serial.print(hxb, 4);
        Serial.print(", hxs=");
        Serial.print(hxs, 4);
        Serial.print(", hyb=");
        Serial.print(hyb, 4);
        Serial.print(", hys=");
        Serial.print(hys, 4);
        Serial.print(", hzb=");
        Serial.print(hzb, 4);
        Serial.print(", hzs=");
        Serial.println(hzs, 4);

        IMU.setMagCalX(hxb, hxs);
        IMU.setMagCalY(hyb, hys);
        IMU.setMagCalZ(hzb, hzs);
        calibrated = true;
        while(1);
    }
    else
    {
        Now = micros();
        deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
        transmit += (Now - lastUpdate);
        lastUpdate = Now;
        IMU.readSensor();
        // read the sensor according to right-hand-rule
        ax = -IMU.getAccelY_mss() / 9.81;
        ay = -IMU.getAccelX_mss() / 9.81;
        az = IMU.getAccelZ_mss() / 9.81;

        gx = IMU.getGyroY_rads();
        gy = IMU.getGyroX_rads();
        gz = -IMU.getGyroZ_rads();

        // X and Y axis are switched;
        mx = IMU.getMagY_uT();
        my = IMU.getMagX_uT();
        mz = -IMU.getMagZ_uT();

        // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
        // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
        // (+ up) of accelerometer and gyro! We have to make some allowance for this
        // orientationmismatch in feeding the output to the quaternion filter. For the
        // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
        // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
        // modified to allow any convenient orientation convention. This is ok by
        // aircraft orientation standards! Pass gyro rate as rad/s
        MadgwickQuaternionUpdate(ax, ay, az, gx, gy, gz, mx, my, mz, deltat);
        //MahonyQuaternionUpdate(ax, ay, az, gx, gy, gz, mx, my, mz, deltat);

        // Define output variables from updated quaternion---these are Tait-Bryan
        // angles, commonly used in aircraft orientation. In this coordinate system,
        // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
        // x-axis and Earth magnetic North (or true North if corrected for local
        // declination, looking down on the sensor positive yaw is counterclockwise.
        // Pitch is angle between sensor x-axis and Earth ground plane, toward the
        // Earth is positive, up toward the sky is negative. Roll is angle between
        // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
        // arise from the definition of the homogeneous rotation matrix constructed
        // from quaternions. Tait-Bryan angles as well as Euler angles are
        // non-commutative; that is, the get the correct orientation the rotations
        // must be applied in the correct order which for this configuration is yaw,
        // pitch, and then roll.
        // For more see
        // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // which has additional links.

        a12 = 2.0f * (*(getQ() + 1) * *(getQ() + 2) + *(getQ()) * *(getQ() + 3));
        a22 = *(getQ()) * *(getQ()) + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3);
        a31 = 2.0f * (*(getQ()) * *(getQ() + 1) + *(getQ() + 2) * *(getQ() + 3));
        a32 = 2.0f * (*(getQ() + 1) * *(getQ() + 3) - *(getQ()) * *(getQ() + 2));
        a33 = *(getQ()) * *(getQ()) - *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3);

        pitch = -asinf(a32);
        roll = atan2f(a31, a33);
        yaw[cnt] = atan2f(a12, a22);

        pitch *= RAD_TO_DEG;
        yaw[cnt] *= RAD_TO_DEG;
        yaw[cnt] += 3.2f; // correction for Munich area
//        if (yaw[cnt] < 0)
//            yaw[cnt]+= 360.0f; // Ensure yaw stays between 0 and 360
        roll *= RAD_TO_DEG;

        cnt = (cnt+1)%FILTER;
        
        lin_ax = ax + a31;
        lin_ay = ay + a32;
        lin_az = az - a33;

        if (transmit >= 17000) // transmit every 17ms (~60Hz)
        {
            float txYaw=0;
            for(int x = 0; x<FILTER ;x++) {
                txYaw += yaw[x];
            }
            txYaw /= FILTER;

            Serial.print(ax, 2);
            Serial.print(", ");
            Serial.print(ay, 2);
            Serial.print(", ");
            Serial.print(az, 2);
            Serial.print(", ");
            Serial.print(gx, 2);
            Serial.print(", ");
            Serial.print(gy, 2);
            Serial.print(", ");
            Serial.print(gz, 2);
            Serial.print(", ");
            Serial.print(mx, 2);
            Serial.print(", ");
            Serial.print(my, 2);
            Serial.print(", ");
            Serial.print(mz, 2);
            Serial.print(", ");
            Serial.print(txYaw, 2);
            Serial.print(", ");
            Serial.print(pitch, 2);
            Serial.print(", ");
            Serial.print(roll, 2);
            Serial.print(", ");
            Serial.print(1. / deltat);
            Serial.print(", ");
            Serial.print(lin_ax, 2);
            Serial.print(", ");
            Serial.print(lin_ay, 2);
            Serial.print(", ");
            Serial.print(lin_az, 2);
            Serial.println();
            transmit = 0;
        }
    }
}