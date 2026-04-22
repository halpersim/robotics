#include <iostream>
#include <cmath> 
#include <wiringPi.h>
#include <stdio.h>
#include <wiringPiSPI.h>
#include <iostream>
#include <stdio.h> 
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "spi_com.h"
#include "ina219.h"
//g++ move_enc.cpp ./ina219.so -o testa_spi $(pkg-config --cflags --libs opencv) -lwiringPi -lpthread -lrobotic_gcc

struct cord{
 int x; 
 int y; 
 int ang; 
}; 

struct Profile {
    double distance;
    double maxVelocity;
    double acceleration;

    double tAccel;
    double tCruise;
    double tDecel;
    double totalTime;
    double peakVelocity;
};

MotorDataType MotorData;

static const int SPI_Channel = 1;

short Des_Speed = 0;
int Select = 0;
int Counter = 0;

const float SHUNT_OHMS = 0.01;
const float MAX_EXPECTED_AMPS = 3.2;
const float Delay_ms = 1000.0;
const int vmax = 6000; 
const int maxAcc = 3000; 


Profile createProfile(double distance, double maxVelocity, double acceleration) {
    Profile p{};
    p.distance = distance;
    p.maxVelocity = maxVelocity;
    p.acceleration = acceleration;

    double tAccel = maxVelocity / acceleration;
    double dAccel = 0.5 * acceleration * tAccel * tAccel;
    double dNeeded = 2.0 * dAccel;

    if (distance > dNeeded) {
        // Trapezoidal profile
        p.tAccel = tAccel;
        p.tDecel = tAccel;

        double dCruise = distance - dNeeded;
        p.tCruise = dCruise / maxVelocity;
        p.totalTime = p.tAccel + p.tCruise + p.tDecel;
        p.peakVelocity = maxVelocity;
    } else {
        // Triangular profile
        p.tCruise = 0.0;
        p.tAccel = std::sqrt(distance / acceleration);
        p.tDecel = p.tAccel;
        p.totalTime = p.tAccel + p.tDecel;
        p.peakVelocity = acceleration * p.tAccel;
    }

    return p;
}

int main() {
	
	wiringPiSetup(); 
	wiringPiSPISetup(SPI_Channel, 1000000);

	INA219 Ina(SHUNT_OHMS, MAX_EXPECTED_AMPS);
	Ina.configure(RANGE_16V, GAIN_8_320MV, ADC_12BIT, ADC_12BIT);
	
	
	cord start{0,0,0}; 
    double end_pos=500000; 
    cord stop{10,10,0};

   

    //calc 
    double dx = stop.x - start.x;
    double dy = stop.y - start.y;

    // dist
    double distance = std::sqrt(dx*dx + dy*dy);

    std::cout << "Distance: " << distance << std::endl;

    // create profile
    Profile p = createProfile(distance, vmax, maxAcc);

    // skriv ut resultat
    std::cout << "Profile results\n";
    std::cout << "---------------\n";
    std::cout << "Distance: " << p.distance << std::endl;
    std::cout << "Max velocity: " << p.maxVelocity << std::endl;
    std::cout << "Acceleration: " << p.acceleration << std::endl;

    std::cout << "tAccel: " << p.tAccel << std::endl;
    std::cout << "tCruise: " << p.tCruise << std::endl;
    std::cout << "tDecel: " << p.tDecel << std::endl;
    std::cout << "Total time: " << p.totalTime << std::endl;
    std::cout << "Peak velocity: " << p.peakVelocity << std::endl;

    // run bbbbbbb
    Send_Read_Motor_Data(&MotorData);// riktiga
	double pos=0; 
    double start_pos = MotorData.Encoder_M1; 
    int counter =0; 
    while(1){
		float Volt = Ina.supply_voltage();
		counter = 1+counter; 
		if (counter >=5000){
			printf("supply voltage = %.2f\n", Volt);
			counter =0; 
			}
		
		if (Volt < 10.5) {
			break;
		}
		if (Volt < 10.8) {
			printf("WARNING LOW VOLTAGE!\n");
		}
		Send_Read_Motor_Data(&MotorData);// riktiga
		pos = MotorData.Encoder_M1-start_pos; 
		if (pos>=end_pos){
			printf("Speed_M1=%d Speed_M2=%d Enkoder_M1= %d Enkoder_M2 %d\n", MotorData.Act_Speed_M1,MotorData.Act_Speed_M2,MotorData.Encoder_M1,MotorData.Encoder_M2);
			printf("%f\n",pos); 
			printf("done bby\n"); 
			MotorData.Set_Speed_M1=0;
			MotorData.Set_Speed_M2=0;
			break; //bby 
			}
		else{
			MotorData.Set_Speed_M1=3000; 
			MotorData.Set_Speed_M2=3000; 
			Send_Read_Motor_Data(&MotorData);// riktiga
			}
		
		
		
		
    } 
    
    return 0;
}
