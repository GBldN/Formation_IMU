/* ################# FORMATION DECOUVERTE DES CENTRALES INERTIELLES 2024 #################
 * Programme exemple N°2 : AJOUT DES CALCULS DE VALEURS REELES A PARTIR DES DONNEES BRUTES
 * 
 * Centrales compatibles :
 *  . Grove-IMU_10DOF_v2.0 : https://wiki.seeedstudio.com/Grove-IMU_10DOF_v2.0/
 *
 * Bibliothèque utilisée :  bibliothèque "Grove_IMU_10DOF_v2.0-master" proposée sur le wiki : https://github.com/Seeed-Studio/Grove_IMU_10DOF_v2.0/
 *  
 * Ce programme permet :
 *    - lire les données brutes RAW fournies par la centrale
 *    - Lire les données flottantes fournies par la bibliothèque de la centrale
 *    - Afficher toutes les valeurs sur le moniteur série
 *  
 * author=gael.balduini@gmail.com
 * licence=CC-by-nc-sa
 * url=https://github.com/GBldN/Formation_IMU
 */

  #include "MPU9250.h"                          //Appel de la bibliothèque de la centrale
  #include "Wire.h"                             //Appel de la bibliothèque de gestion de la communication I2C
  #include "I2Cdev.h"
  
  MPU9250 accelgyro;                            //Déclaration de la méthode "MyIMU" pour l'utilisation de la centrale
  I2Cdev   I2C_M;


  int16_t  ax_brut, ay_brut, az_brut;           //Variables pour la lecture des valeurs brutes d'accélérations (accéléromètre)
  int16_t  gx_brut, gy_brut, gz_brut;           //Variables pour la lecture des valeurs brutes des vitesses angulaires (gyromètre)
  float    Ax,Ay,Az,Gx,Gy,Gz;                   //Variables pour le calcul des valeurs réelles

/* #### ROUTINE D'INITIALISATION  (exécutée une seule fois) #### */
void setup() 
{
  Wire.begin();
  Wire.setClock(400000);                        //400kHz I2C clock
  Serial.begin(115200);                         //démarrage de la fonction de communication série pour affichage sur le moniteur série
  
  /* -- Démarrage de la centrale et test de bon fonctionnement -- */
  accelgyro.initialize();
  if (accelgyro.testConnection() != 0) {
      Serial.println("Problème de connexion avec la centrale");
  } else {
      Serial.println("Centrale prête à fonctionner ! ");
  }
}

/* #### BOUCLE PRINCIPALE (exécutée en permanence) #### */
void loop() 
{
  /* -- Lecture des valeurs brutes -- */
  accelgyro.getMotion6(&ax_brut, &ay_brut, &az_brut, &gx_brut, &gy_brut, &gz_brut);
 
  /* -- Lecture des valeurs réelles de la bibliothèque -- */
  float ax_biblio = 0;  //Non disponible dans la bibliothèque MPU9250
  float ay_biblio = 0;
  float az_biblio = 0;
  float gx_biblio = 0;
  float gy_biblio = 0;
  float gz_biblio = 0;
   
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
