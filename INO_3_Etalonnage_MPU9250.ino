/* ################# FORMATION DECOUVERTE DES CENTRALES INERTIELLES 2024 #################
 * Programme exemple N°3 : AJOUT DE LA ROUTINE D'ETALONNAGE
 * 
 * Centrales compatibles :
 *  . Grove-IMU_10DOF_v2.0 : https://wiki.seeedstudio.com/Grove-IMU_10DOF_v2.0/
 *  . Grove-IMU_9DOF       : https://wiki.seeedstudio.com/Grove-IMU_9DOF-lcm20600+AK09918/
 *  . toutes les centrales équipées de la puce MP6050 :  https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/ 
 *  
 * Bibliothèque utilisée :  bibliothèque "Grove_IMU_10DOF_v2.0-master" proposée sur le wiki : https://github.com/Seeed-Studio/Grove_IMU_10DOF_v2.0/
 *  
 * Ce programme permet :
 *    - Lire les données brutes fournies par la bibliothèque de la centrale (dans une routine séparée)
 *    - Réaliser un étalonnage au démarrage de la centrale pour évaluers les valeurs d'offsets.
 *    - Calculer les valeurs réelles prennant en compte les valeurs d'offsets.
 *    - Afficher toutes les valeurs sur le moniteur série.
 * 
 * Remarque par rapport au programme précédent : L'acquisition des valeurs brutes est maintenant déplacée dans une routine séparée puisqu'elle est appelée plusieurs fois.
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

  #define sensi_acc      61                     //Sensibilité de l'accéléromètre en micro g/LSB   (réglé par défault pour 16 g     calculé en fonction de la pleine échelle)
  #define sensi_gyr    8750                     //Sensibilité du gyromètre       en micro dps/LSB (réglé par défault pour 2000 °/s calculé en fonction de la pleine échelle)
  
  #include "MPU9250.h"                          //Appel de la bibliothèque de la centrale
  #include "Wire.h"                             //Appel de la bibliothèque de gestion de la communication I2C
  #include "I2Cdev.h"
  
  MPU9250 accelgyro;                            //Déclaration de la méthode "MyIMU" pour l'utilisation de la centrale
  I2Cdev   I2C_M;
  
  int16_t  ax_brut, ay_brut, az_brut;           //Variables pour la lecture des valeurs brutes d'accélérations (accéléromètre)
  int16_t  gx_brut, gy_brut, gz_brut;           //Variables pour la lecture des valeurs brutes des vitesses angulaires (gyromètre)
  int32_t  ax_offset, ay_offset, az_offset;     //Variables pour le stockage des valeurs d'offsets de l'accéléromètre
  int32_t  gx_offset, gy_offset, gz_offset;     //Variables pour le stockage des valeurs d'offsets du gyromètre
  float    Ax,Ay,Az,Gx,Gy,Gz;                   //Variables pour le calcul des valeurs réelles
 
/* #### ROUTINE D'INITIALISATION  (exécutée une seule fois) #### */
void setup() 
{
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);                         //démarrage de la fonction de communication série pour affichage sur le moniteur série
  /* -- Démarrage de la centrale et test de bon fonctionnement -- */
  accelgyro.initialize();
  if (accelgyro.testConnection() != 0) {
      Serial.println("Problème de connexion avec la centrale");
  } else {
      Serial.println("Centrale prête à fonctionner ! ");
  }

 etalonnage();                                  //Appel de la routine d'étalonnage
}

/* #### BOUCLE PRINCIPALE (exécutée en permanence) #### */
void loop() 
{ 
  lecture_valeurs_brutes();                     //Appel de la routine de lecture des valeurs brutes
  
  /* -- Calculs des valeurs réelles -- */
  Ax = float(ax_brut - ax_offset ) * sensi_acc / 1000000;    //Calcul des valeurs d'accélérations en g en fonction de la sensibilité du capteur
  Ay = float(ay_brut - ay_offset ) * sensi_acc / 1000000;    //Les valeurs brutes sont codées sur 16 bits (0 et 65535)
  Az = float(az_brut - az_offset ) * sensi_acc / 1000000;    //La sensibilité du capteur dépend de la valeur pleine échelle choisie en haut et définie dans les réglages du capteur
  Gx = float(gx_brut - gx_offset ) * sensi_gyr / 1000000;    //Calcul des valeurs de vitesses angulaires en °/s
  Gy = float(gy_brut - gy_offset ) * sensi_gyr / 1000000;
  Gz = float(gz_brut - gz_offset ) * sensi_gyr / 1000000;

  /* -- Affichage des valeurs lues par la centrale -- */
  Serial.println("  -- Accéléromètre : --");  
  Serial.print("  X_raw = "); Serial.print(ax_brut);  Serial.print(" g"); Serial.print(" - X_calc = "); Serial.print(Ax,2); Serial.println(" g");
  Serial.print("  Y_raw = "); Serial.print(ay_brut);  Serial.print(" - Y_calc = "); Serial.print(Ay,2); Serial.println(" g");
  Serial.print("  Z_raw = "); Serial.print(az_brut);  Serial.print(" - Z_calc = "); Serial.print(Az,2); Serial.println(" g");
  Serial.println("  --   Gyromètre :   --");
  Serial.print("  X_raw = "); Serial.print(gx_brut);  Serial.print(" - X_calc = "); Serial.print(Gx,2); Serial.println(" °/s");
  Serial.print("  Y_raw = "); Serial.print(gy_brut);  Serial.print(" - Y_calc = "); Serial.print(Gy,2); Serial.println(" °/s");
  Serial.print("  Z_raw = "); Serial.print(gz_brut);  Serial.print(" - Z_calc = "); Serial.print(Gz,2); Serial.println(" °/s");
  Serial.println();
  Serial.println();
  delay(500);
}

/****************************************************
 *      ROUTINE DE LECTURE DES VALEURS BRUTES       *
 ****************************************************/
void lecture_valeurs_brutes()
{
  /* -- Lecture des valeurs brutes -- */
 accelgyro.getMotion6(&ax_brut, &ay_brut, &az_brut, &gx_brut, &gy_brut, &gz_brut);
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
