/* ################# FORMATION DECOUVERTE DES CENTRALES INERTIELLES 2024 #################
 * Programme exemple N°4 : AJOUT DES CALCULS DES ANGLES DE ROULIS ET DE TANGAGE AVEC L'ACCELEROMETRE
 * 
 * Lecture de la centrale Grove_IMU_6 DOF : https://wiki.seeedstudio.com/Grove-6-Axis_AccelerometerAndGyroscope/
 * avec la bibliothèque "Seeed_Arduino_LSM6DS3-master.zip" proposée sur le wiki : https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3
 *  
 * Ce programme permet :
 *    - Lire les données brutes fournies par la bibliothèque de la centrale (dans une routine séparée)
 *    - Réaliser un étalonnage au démarrage de la centrale afin de compenser les valeurs d'offsets.
 *    - Calculer les valeurs réelles prennant en compte les valeurs d'offsets.
 *    - Calculer les angles de ROULIS et TANGAGES avec l'accéléromètre
 *    - Afficher toutes les valeurs sur le moniteur série
 * 
 * Remarque par rapport au programme précédent : Suppression du délai en fin de programme
 * 
 * les sensibilitées sont données dans les datasheets en fonction du réglage de la valeur pleine échelle. 
 *  Composant | valeur pleine échelle  ||            sensibilité             |
 *            |                        ||      LSM6DS3    ||      MPU9250    |
 *  ----------+------------------------++-----------------++-----------------|
 *            |         ±2g            ||  0.061 mg/LSB   || 16384   LSB/g   |
 *   accéléro |         ±4g            ||  0.122 mg/LSB   ||  8192   LSB/g   |
 *    mètre   |         ±8g            ||  0.244 mg/LSB   ||  4096   LSB/g   |
 *            |         ±16g           ||  0.488 mg/LSB   ||  2048   LSB/g   |
 *  ----------+------------------------++-----------------++-----------------|
 *            |  ±125°/s ( ±21 tr/min) ||  4.375 mdps/LSB ||    Non inclu    |
 *    gyro    |  ±250°/s ( ±42 tr/min) ||  8.75  mdps/LSB ||   131   LSB/dps |
 *    mètre   |  ±500°/s ( ±83 tr/min) || 17.5   mdps/LSB ||    65.5 LSB/dps |
 *            | ±1000°/s (±167 tr/min) || 35     mdps/LSB ||    32.8 LSB/dps |
 *            | ±2000°/s (±333 tr/min) || 70     mdps/LSB ||    16.4 LSB/dps |
 * 
 * Attention pour la puce LSM6DS3 les valeurs données ne sont toujours pas celles que l'on pourrait calculer exemple pour le gyromètre :
 * 4370 µdps/LSB - ATTENTION PAR CALCUL = 3814.75
 * 8750 µdps/LSB - ATTENTION PAR CALCUL = 7476.92
 * 17500 µdps/LSB - ATTENTION PAR CALCUL = 15259
 * 35000 µdps/LSB - ATTENTION PAR CALCUL = 30518
 * 70000 µdps/LSB - ATTENTION PAR CALCUL = 61036.1
 * 
 * author=gael.balduini@gmail.com
 * licence=CC-by-nc-sa
 * url=https://github.com/GBldN/Formation_IMU
 */

  #define sensi_acc     488                   //Sensibilité de l'accéléromètre en milli g/LSB   (réglé par défault pour 16 g     calculé en fonction de la pleine échelle)
  #define sensi_gyr   70000                   //Sensibilité du gyromètre       en milli dps/LSB (réglé par défault pour 2000 °/s calculé en fonction de la pleine échelle)

  #include "LSM6DS3.h"                        //Appel de la bibliothèque de la centrale
  #include "Wire.h"                           //Appel de la bibliothèque de gestion de la communication I2C
  
  LSM6DS3 myIMU(I2C_MODE, 0x6A);              //Déclaration de la méthode "MyIMU" pour l'utilisation de la centrale

  int16_t  ax_brut, ay_brut, az_brut;         //Variables pour la lecture des valeurs brutes d'accélérations (accéléromètre)
  int16_t  gx_brut, gy_brut, gz_brut;         //Variables pour la lecture des valeurs brutes des vitesses angulaires (gyromètre)
  int32_t  ax_offset, ay_offset, az_offset;   //Variables pour le stockage des valeurs d'offsets de l'accéléromètre
  int32_t  gx_offset, gy_offset, gz_offset;   //Variables pour le stockage des valeurs d'offsets du gyromètre
  float    Ax,Ay,Az,Gx,Gy,Gz;                 //Variables pour le calcul des valeurs réelles

  float roulis_acc, tangage_acc;              //Variables pour le calcul des angles d'inclinaisons avec l'accéléromètre : ROULIS et TANGAGE
  
/* #### ROUTINE D'INITIALISATION  (exécutée une seule fois) #### */
void setup() 
{
  Serial.begin(115200);                       //démarrage de la fonction de communication série pour affichage sur le moniteur série
  /* -- Démarrage de la centrale et test de bon fonctionnement -- */
  if (myIMU.begin() != 0) {
      Serial.println("Problème de connexion avec la centrale");
  } else {
      Serial.println("Centrale prête à fonctionner ! ");
  }

  etalonnage();                               //Appel de la routine d'étalonnage
}

/* #### BOUCLE PRINCIPALE (exécutée en permanence) #### */
void loop() 
{ 
  lecture_valeurs_brutes();                                  //Appel de la routine de lecture des valeurs brutes
  
  /* -- Calculs des valeurs réelles -- */
  Ax = float(ax_brut - ax_offset ) * sensi_acc / 1000000;    //Calcul des valeurs d'accélérations en g en fonction de la sensibilité du capteur
  Ay = float(ay_brut - ay_offset ) * sensi_acc / 1000000;    //Les valeurs brutes sont codées sur 16 bits (0 et 65535)
  Az = float(az_brut - az_offset ) * sensi_acc / 1000000;    //La sensibilité du capteur dépend de la valeur pleine échelle choisie en haut et définie dans les réglages du capteur
  Gx = float(gx_brut - gx_offset ) * sensi_gyr / 1000000;    //Calcul des valeurs de vitesses angulaires en °/s
  Gy = float(gy_brut - gy_offset ) * sensi_gyr / 1000000;
  Gz = float(gz_brut - gz_offset ) * sensi_gyr / 1000000;

  /* -- Calcul des angles de ROULIS et TANGAGE à partir de l'accéléromètre -- */
    //CHOIX N°1 LACET autour de Z puis ROULIS autour de X puis TANGAGE autour de Y
  if ( Az >= 0) roulis_acc  = atan2( Ay ,   sqrt( pow(Ax, 2) + pow(Az, 2)) ) * RAD_TO_DEG;
  else          roulis_acc  = atan2( Ay , - sqrt( pow(Ax, 2) + pow(Az, 2)) )  * RAD_TO_DEG;
  
                tangage_acc = atan2( - Ax , Az ) * RAD_TO_DEG;
  
  /* -- Affichage des valeurs lues par la centrale -- */
  //Serial.print("ACC ( ");    Serial.print(Ax,1); Serial.print(" , "); Serial.print(Ay,1);  Serial.print(" , ");  Serial.print(Az,1); Serial.print(" ) g ");
  //Serial.print(" | GYR ( "); Serial.print(Gx,0); Serial.print(", ");  Serial.print(Gy,0);  Serial.print(" , ");  Serial.print(Gz,0); Serial.print(" ) deg/s");
  
  Serial.print(" | ROULIS Acc = ");  Serial.print(roulis_acc , 0);  Serial.print(" °");
  
  Serial.print(" | TANGAGE_Acc = "); Serial.print(tangage_acc , 0); Serial.print(" °"); 
  
  Serial.println();
}

/****************************************************
 *      ROUTINE DE LECTURE DES VALEURS BRUTES       *
 ****************************************************/
void lecture_valeurs_brutes()
{
  /* -- Lecture des valeurs brutes -- */
  ax_brut = myIMU.readRawAccelX();
  ay_brut = myIMU.readRawAccelY();
  az_brut = myIMU.readRawAccelZ();
  gx_brut = myIMU.readRawGyroX();
  gy_brut = myIMU.readRawGyroY();
  gz_brut = myIMU.readRawGyroZ();
}


/****************************************************
 *      ROUTINE D'ETALONNAGE calcul des offsets     *
 ****************************************************/
void etalonnage()
{
  int nombre_echantillons = 1000; //Variable locale pour définir le nombre d'itération du calcul
  
  //Réinitialisation des offsets
  ax_offset = 0;
  ay_offset = 0;
  az_offset = 0;
  gx_offset = 0;
  gy_offset = 0;
  gz_offset = 0;
  
  //Boucle de sommation de chaque valeur pour le calcul de la moyenne
  for (int i = 1; i <= nombre_echantillons; i++) 
  {
    lecture_valeurs_brutes();
    ax_offset += ax_brut;
    ay_offset += ay_brut;
    az_offset += az_brut;
    gx_offset += gx_brut;
    gy_offset += gy_brut;
    gz_offset += gz_brut;
  } 
  
  //Calcul de la moyenne des valeurs mesurées pendant les itérations
  ax_offset /= nombre_echantillons;
  ay_offset /= nombre_echantillons;
  az_offset /= nombre_echantillons;
  gx_offset /= nombre_echantillons;
  gy_offset /= nombre_echantillons;
  gz_offset /= nombre_echantillons;
  
  //Rectification de l'offset sur Z pour qu'il donne 1g en statique (fonction de la sensibilité)
  az_offset -= 1000000 / sensi_acc; 
}
