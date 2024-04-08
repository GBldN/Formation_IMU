/* ################# FORMATION DECOUVERTE DES CENTRALES INERTIELLES 2024 #################
 * Programme exemple N°2 : AJOUT DES CALCULS DE VALEURS REELES A PARTIR DES DONNEES BRUTES
 * 
 * Lecture de la centrale Grove_IMU_6 DOF : https://wiki.seeedstudio.com/Grove-6-Axis_AccelerometerAndGyroscope/
 * avec la bibliothèque "Seeed_Arduino_LSM6DS3-master.zip" proposée sur le wiki : https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3
 *  
 * Ce programme permet :
 *    - lire les données brutes RAW fournies par la centrale
 *    - Lire les données flottantes fournies par la bibliothèque de la centrale
 *    - Calculer les valeurs réelles à partir des données brutes
 *    - Afficher toutes les valeurs sur le moniteur série
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
  float    Ax,Ay,Az,Gx,Gy,Gz;                 //Variables pour le calcul des valeurs réelles

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
}

/* #### BOUCLE PRINCIPALE (exécutée en permanence) #### */
void loop() 
{
  /* -- Lecture des valeurs brutes -- */
  ax_brut = myIMU.readRawAccelX();
  ay_brut = myIMU.readRawAccelY();
  az_brut = myIMU.readRawAccelZ();
  gx_brut = myIMU.readRawGyroX();
  gy_brut = myIMU.readRawGyroY();
  gz_brut = myIMU.readRawGyroZ();

  /* -- Lecture des valeurs de la bibliothèque -- */
  float ax_biblio = myIMU.readFloatAccelX();
  float ay_biblio = myIMU.readFloatAccelY();
  float az_biblio = myIMU.readFloatAccelZ();
  float gx_biblio = myIMU.readFloatGyroX();
  float gy_biblio = myIMU.readFloatGyroY();
  float gz_biblio = myIMU.readFloatGyroZ();
  
  /* -- Calculs des valeurs -- */
  Ax = float(ax_brut ) * sensi_acc / 1000000;    //Calcul des valeurs d'accélérations en g en fonction de la sensibilité du capteur
  Ay = float(ay_brut ) * sensi_acc / 1000000;    //Les valeurs brutes sont codées sur 16 bits (0 et 65535)
  Az = float(az_brut ) * sensi_acc / 1000000;    //La sensibilité du capteur dépend de la valeur pleine échelle choisie en haut et définie dans les réglages du capteur
  Gx = float(gx_brut ) * sensi_gyr / 1000000;    //Calcul des valeurs de vitesses angulaires en °/s
  Gy = float(gy_brut ) * sensi_gyr / 1000000;
  Gz = float(gz_brut ) * sensi_gyr / 1000000;

  
  /* -- Affichage des valeurs lues par la centrale -- */
  Serial.println("  -- Accéléromètre : --");  
  Serial.print("  X_raw = "); Serial.print(ax_brut);  Serial.print(" - X_réel = "); Serial.print(ax_biblio,2); Serial.print(" g"); Serial.print(" - X_calc = "); Serial.print(Ax,2); Serial.println(" g");
  Serial.print("  Y_raw = "); Serial.print(ay_brut);  Serial.print(" - Y_réel = "); Serial.print(ay_biblio,2); Serial.print(" g"); Serial.print(" - Y_calc = "); Serial.print(Ay,2); Serial.println(" g");
  Serial.print("  Z_raw = "); Serial.print(az_brut);  Serial.print(" - Z_réel = "); Serial.print(az_biblio,2); Serial.print(" g"); Serial.print(" - Z_calc = "); Serial.print(Az,2); Serial.println(" g");
  Serial.println("  --   Gyromètre :   --");
  Serial.print("  X_raw = "); Serial.print(gx_brut);  Serial.print(" - X_réel = "); Serial.print(gx_biblio,2); Serial.print(" °/S"); Serial.print(" - X_calc = "); Serial.print(Gx,2); Serial.println(" °/s");
  Serial.print("  Y_raw = "); Serial.print(gy_brut);  Serial.print(" - Y_réel = "); Serial.print(gy_biblio,2); Serial.print(" °/S"); Serial.print(" - Y_calc = "); Serial.print(Gy,2); Serial.println(" °/s");
  Serial.print("  Z_raw = "); Serial.print(gz_brut);  Serial.print(" - X_réel = "); Serial.print(gz_biblio,2); Serial.print(" °/S"); Serial.print(" - Z_calc = "); Serial.print(Gz,2); Serial.println(" °/s");
  Serial.println();
  Serial.println();
  delay(500);
}
