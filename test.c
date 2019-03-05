#include "SparkFunLSM9DS1.h"
#include <sys/time.h>
//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

void millis()
{
    struct timespec ts;
    gettimeofday(&tv, NULL);
    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000L);
    
}

void printGyro()
{
    // Now we can use the gx, gy, and gz variables as we please.
    // Either print them as raw ADC values, or calculated in DPS.
    printf("G: ");
    #ifdef PRINT_CALCULATED
    // If you want to print calculated values, you can use the
    // calcGyro helper function to convert a raw ADC value to
    // DPS. Give the function the value that you want to convert.
        printf("%.2f, %.2f, %.2f", imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz));
        printf(" deg/s\n");
    #elif defined PRINT_RAW
         printf("%.2f, %.2f, %.2f\n", imu.gx, imu.gy, imu.gz);
    #endif
}

void printAccel()
{  
    // Now we can use the ax, ay, and az variables as we please.
    // Either print them as raw ADC values, or calculated in g's.
    printf("A: ");
    #ifdef PRINT_CALCULATED
    // If you want to print calculated values, you can use the
    // calcAccel helper function to convert a raw ADC value to
    // g's. Give the function the value that you want to convert.
        printf("%.2f, %.2f, %.2f", imu.calcGyro(imu.ax), imu.calcGyro(imu.ay), imu.calcGyro(imu.az));
        printf(" g\n");
    #elif defined PRINT_RAW 
        printf("%.2f, %.2f, %.2f\n", imu.ax, imu.ay, imu.az);
    #endif

}

void printMag()
{  
    // Now we can use the mx, my, and mz variables as we please.
    // Either print them as raw ADC values, or calculated in Gauss.
    printf("M: ");
    #ifdef PRINT_CALCULATED
    // If you want to print calculated values, you can use the
    // calcMag helper function to convert a raw ADC value to
    // Gauss. Give the function the value that you want to convert.
        printf("%.2f, %.2f, %.2f", imu.calcGyro(imu.mx), imu.calcGyro(imu.my), imu.calcGyro(imu.mz));
        printf(" gauss\n");
    #elif defined PRINT_RAW
        printf("%.2f, %.2f, %.2f\n", imu.mx, imu.my, imu.mz);
    #endif
}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));
    
    float heading;
    if (my == 0)
        heading = (mx < 0) ? PI : 0;
    else
        heading = atan2(mx, my);
        
    heading -= DECLINATION * PI / 180;
    
    if (heading > PI) heading -= (2 * PI);
    else if (heading < -PI) heading += (2 * PI);
    
    // Convert everything from radians to degrees:
    heading *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll  *= 180.0 / PI;
    
    printf("Pitch, Roll: %.2f, %.2f; Heading: .2f\n", pitch, roll, heading);
}

int main() {
    LSM9DS1 imu(IMU_MODE_I2C, 0x6B, 0x1E, 1);

    imu.settings.device.commInterface = IMU_MODE_I2C;
    imu.settings.device.mAddress = LSM9DS1_M;
    imu.settings.device.agAddress = LSM9DS1_AG;

    if (!imu.begin()){
        printf("Failed to communicate with LSM9DS1.\n");
        printf("Double-check wiring.\n");
        printf("Default settings in this sketch will work for an out of the box LSM9DS1 Breakout, but may need to be modified if the board jumpers are.\n");
        while (1);
    }

    while(1){
        if ( imu.gyroAvailable() ){
        // To read from the gyroscope,  first call the
        // readGyro() function. When it exits, it'll update the
        // gx, gy, and gz variables with the most current data.
            imu.readGyro();
        }
        if ( imu.accelAvailable() ){
        // To read from the accelerometer, first call the
        // readAccel() function. When it exits, it'll update the
        // ax, ay, and az variables with the most current data.
            imu.readAccel();
        }
        if ( imu.magAvailable() ){
        // To read from the magnetometer, first call the
        // readMag() function. When it exits, it'll update the
        // mx, my, and mz variables with the most current data.
            imu.readMag();
        }
    
        if ((lastPrint + PRINT_SPEED) < millis()){
            printGyro();  // Print "G: gx, gy, gz"
            printAccel(); // Print "A: ax, ay, az"
            printMag();   // Print "M: mx, my, mz"
        // Print the heading and orientation for fun!
        // Call print attitude. The LSM9DS1's mag x and y
        // axes are opposite to the accelerometer, so my, mx are
        // substituted for each other.
            printAttitude(imu.ax, imu.ay, imu.az, 
                    -imu.my, -imu.mx, imu.mz);
            printf("\n");
            lastPrint = millis(); // Update lastPrint time
        }
    } 
}