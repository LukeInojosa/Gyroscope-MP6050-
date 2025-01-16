
#include "mbed.h"
#include "MPU6050.h"
#include <chrono>

# define M_PI 3.14159265358979323846	/* pi */

Ticker tikerGiroscopy;
Timer timerGiroscopy;

bool _canReadGiroscopy = false; // flag que controla a leitura do giroscópio
int _periodToReadGiroscopy = 1000; // tempo entre leituras sucessivas do giroscópio
bool _activeIMU = false; // indica se a placa está ativa com sucesso

// ******* EXTERNAL MPU 6050 ******
// Needs to cut PF_0 PIN in main board!!!
#define MPU_SDA PF_0 // I2C
#define MPU_SCL PF_1 // I2C
MPU6050 _MPU(MPU_SDA, MPU_SCL);
bool _activeIMU =_MPU.initialize();
double _gyroOffset[3] = {0,0,0};

void canReadGiroscopy(){
    if(!_activeIMU) _canReadGiroscopy = false;
    else  _canReadGiroscopy = true;
}

// Determina os Offsets iniciais do giroscópio
bool calibrateGiroscopy(){
    if(!_activeIMU) return false;
    while(true){
        for(int i = 0;i<100;i++){
            
        }
    }
}

double _DeltaAngle;
double _angle;

// Realiza a leitura do giroscópio e atualiza:
// velocidades: _Dangle (rad/s)
// angulo atual do giroscópio: _angle (rad)
double readGiroscopy(){

    // realiza a leitura do giroscópio
    double gyro[3] = {0, 0, 0};
    _MPU.readGyro(gyro);

    // converte velocidade angular de grau/s para rad/s 
    _DeltaAngle = (gyro[2] - _gyroOffset[2])*(M_PI/180);

     // calcula tempo decorrido desde a ultima atualização do ângulo
    timerGiroscopy.stop();
    double time_seconds = (chrono::duration_cast<chrono::milliseconds>(timerGiroscopy.elapsed_time()).count())/1000;

    // atualiza deslocamento do angulo 
    _angle += _DeltaAngle*time_seconds;

}

int main(void){
    //calibra os offsets do giroscópio 
    bool _isCalibrateGiroscopy = calibrateGiroscopy();

    //determinar periodos fixos para realizar a leitura do giroscópio
    tikerGiroscopy.attach(&canReadGiroscopy, _periodToReadGiroscopy);

    timerGiroscopy.start();

    while(true){
        if(_isCalibrateGiroscopy && _canReadGiroscopy){
            // lê giroscópio
            readGiroscopy();

            timerGiroscopy.start();
            _canReadGiroscopy = false;

        }
    }
}