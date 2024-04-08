/* ################# FORMATION DECOUVERTE DES CENTRALES INERTIELLES 2024 #################
 * Programme exemple N°1 : TEST DE LA CENTRALE AVEC LA BIBLIOTHEQUE
 * 
 * Test de lecture de la centrale Grove_IMU_6 DOF : https://wiki.seeedstudio.com/Grove-6-Axis_AccelerometerAndGyroscope/
 * avec la bibliothèque "Seeed_Arduino_LSM6DS3-master.zip" proposée sur le wiki : https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3
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

  #include "LSM6DS3.h"                        //Appel de la bibliothèque de la centrale
  #include "Wire.h"                           //Appel de la bibliothèque de gestion de la communication I2C
  
  LSM6DS3 myIMU(I2C_MODE, 0x6A);              //Déclaration de la méthode "MyIMU" pour l'utilisation de la centrale

  int16_t       ax_brut, ay_brut, az_brut;    //Variables pour la lecture des valeurs brutes d'accélérations (accéléromètre)
  int16_t       gx_brut, gy_brut, gz_brut;    //Variables pour la lecture des valeurs brutes des vitesses angulaires (gyromètre)

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
  ax_brut = myIMU.readFloatAccelX();
  ay_brut = myIMU.readFloatAccelY();
  az_brut = myIMU.readFloatAccelZ();
  gx_brut = myIMU.readFloatGyroX();
  gx_brut = myIMU.readFloatGyroY();
  gx_brut = myIMU.readFloatGyroZ();
 
  /* -- Lecture des valeurs de la bibliothèque -- */
  float ax_biblio = myIMU.readFloatAccelX();
  float ay_biblio = myIMU.readFloatAccelY();
  float az_biblio = myIMU.readFloatAccelZ();
  float gx_biblio = myIMU.readFloatGyroX();
  float gy_biblio = myIMU.readFloatGyroY();
  float gz_biblio = myIMU.readFloatGyroZ();
   
  /* -- Affichage des valeurs lues par la centrale -- */
  Serial.println("  -- Accéléromètre : --");  
  Serial.print("  X_raw = "); Serial.print(ax_brut);  Serial.print(" - X_réel = "); Serial.print(ax_biblio,2); Serial.println(" g"); 
  Serial.print("  Y_raw = "); Serial.print(ay_brut);  Serial.print(" - Y_réel = "); Serial.print(ay_biblio,2); Serial.println(" g");
  Serial.print("  Z_raw = "); Serial.print(az_brut);  Serial.print(" - Z_réel = "); Serial.print(az_biblio,2); Serial.println(" g");
  Serial.println("  --   Gyromètre :   --");
  Serial.print("  X_raw = "); Serial.print(gx_brut);  Serial.print(" - X_réel = "); Serial.print(gx_biblio,2); Serial.println(" °/S");
  Serial.print("  Y_raw = "); Serial.print(gy_brut);  Serial.print(" - Y_réel = "); Serial.print(gy_biblio,2); Serial.println(" °/S");
  Serial.print("  Z_raw = "); Serial.print(gz_brut);  Serial.print(" - X_réel = "); Serial.print(gz_biblio,2); Serial.println(" °/S");
  Serial.println();
  Serial.println();  delay(500);
}
