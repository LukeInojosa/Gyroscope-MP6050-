
#include "mbed.h"
#include "MPU6050.h"
#include <chrono>
#include <cmath>
#include <thread>


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
bool _activeIMU;
double _gyroOffset[3] = {0,0,0};

void canReadGiroscopy(){
    if(!_activeIMU) _canReadGiroscopy = false;
    else  _canReadGiroscopy = true;
}

//Lê os registradores do giroscópio e já acrescente o offset
void readGyro(double* gyroReadings){
    _MPU.readGyro(gyroReadings);
    gyroReadings[0] -= _gyroOffset[0];
    gyroReadings[1] -= _gyroOffset[1];
    gyroReadings[2] -= _gyroOffset[2];
}

// Determina os Offsets iniciais do giroscópio
bool calibrateGiroscopy(int num_samples, int time_between_samples, double treshold){
    if(!_activeIMU) return false;

    double gyro[3] = {0, 0, 0};
    double mean[3] = {0, 0, 0};
    int ready = 0;

    while(ready < 3){
        ready = 0;

        mean[0] = 0;
        mean[1] = 0;
        mean[2] = 0;

        // lê num_samples amostras do giroscópio
        for(int i = 0;i<num_samples;i++){

            // lê informação do giroscópio , já retirando o offset
            readGyro(gyro);
            
            // as leituras são acumuladas
            mean[0] += gyro[0];
            mean[1] += gyro[1];
            mean[2] += gyro[2];

            // este intervalo é dado para que as leituras não sejam repetidas
            ThisThread::sleep_for(chrono::milliseconds(time_between_samples));
        }

        // é retirada as médias das leituras
        mean[0] /= num_samples;
        mean[1] /= num_samples;
        mean[2] /= num_samples;

        // se a média da leitura não estiver abaixo do limite aceitável
        // então damos um pequeno incremento no offset do giroscópio
        if(fabs(mean[0]) < treshold ) ready += 1;
        else _gyroOffset[0] + mean[0]/(treshold+1);
        if(fabs(mean[1]) < treshold ) ready += 1;
        else _gyroOffset[1] + mean[1]/(treshold+1);
        if(fabs(mean[2]) < treshold ) ready += 1;
        else _gyroOffset[2] + mean[2]/(treshold+1);
    }
}

double _DeltaAngle;
double _angle;

// Realiza a leitura do giroscópio e atualiza:
// velocidades: _Dangle (rad/s)
// angulo atual do giroscópio: _angle (rad)
double readGiroscopy(int n_mean){

    double gyro[3] = {0, 0, 0};
    double out = 0;

    // realiza a leitura do giroscópio
    for (int i=0; i< n_mean; i++){
        readGyro(gyro);
        out += gyro[2];
    }
    out /= n_mean;
    
    // converte velocidade angular de grau/s para rad/s 
    _DeltaAngle = out*(M_PI/180);

     // calcula tempo decorrido desde a ultima atualização do ângulo
    timerGiroscopy.stop();
    double time_seconds = (chrono::duration_cast<chrono::milliseconds>(timerGiroscopy.elapsed_time()).count())/1000;

    // atualiza deslocamento do angulo 
    _angle += _DeltaAngle*time_seconds;

    // retorna a contagem do timer
    timerGiroscopy.start();

}

int main(void){

    while(!_activeIMU){
        ThisThread::sleep_for(chrono::milliseconds(10));
        _activeIMU = _MPU.initialize();
    }
    
    //calibra os offsets do giroscópio 
    bool _isCalibrateGiroscopy = calibrateGiroscopy(100,5,0.2);

    //determinar periodos fixos para realizar a leitura do giroscópio
    tikerGiroscopy.attach(&canReadGiroscopy, _periodToReadGiroscopy);

    timerGiroscopy.start();

    while(true){
        if(_isCalibrateGiroscopy && _canReadGiroscopy){
            // lê giroscópio
            readGiroscopy();
            _canReadGiroscopy = false;
        }
    }
}