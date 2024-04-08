/* ################# FORMATION DECOUVERTE DES CENTRALES INERTIELLES 2024 #################
 * Programme exemple N°9 : UTILISATION DU FILTRE DE MADGWICK
 * 
 * Lecture de la centrale Grove_IMU_6 DOF : https://wiki.seeedstudio.com/Grove-6-Axis_AccelerometerAndGyroscope/
 * avec la bibliothèque "Seeed_Arduino_LSM6DS3-master.zip" proposée sur le wiki : https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3
 *  
 * Ce programme permet :
 *    - Lire les données flottantes fournies par la bibliothèque de la centrale (dans une routine séparée
 *    - Réaliser un étalonnage au démarrage de la centrale afin de compenser les valeurs d'offsets.
 *    - Calculer les valeurs réelles prennant en compte les valeurs d'offsets.
 *    - Calculer les angles de ROULIS et TANGAGES avec l'accéléromètre
 *    - Calculer les angles de ROULIS et TANGAGES avec le gyromètre (AVEC CORRECTION DU A L'INCLINAISON DE LA CENTRALE)
 *    - Moduler les angles calculés par le gyromètre à ±180°
 *    - Filtrage des 2 valeurs par filtrage de Madgick
 *    - Afficher toutes les valeurs sur le moniteur série
 * 
 * Remarque par rapport au programme précédent : Remplacement du iltrage complémentaire par la bibliothèque MADGWICK
 * 
 * les sensibilitées sont données dans la datasheet en fonction du réglage de la valeur pleine échelle.
 *  Composant | valeur pleine échelle  | sensibilité
 *  ----------+------------------------+---------------
 *            |         ±2g            |    61 µg/LSB
 *   accéléro |         ±4g            |   122 µg/LSB
 *    mètre   |         ±8g            |   244 µg/LSB
 *            |         ±16g           |   488 µg/LSB
 *  ----------+------------------------+---------------
 *            |  ±125°/s ( ±21 tr/min) |  4370 µdps/LSB - ATTENTION PAR CALCUL = 3814.75
 *    gyro    |  ±250°/s ( ±42 tr/min) |  8750 µdps/LSB - ATTENTION PAR CALCUL = 7476.92
 *    mètre   |  ±500°/s ( ±83 tr/min) | 17500 µdps/LSB - ATTENTION PAR CALCUL = 15259
 *            | ±1000°/s (±167 tr/min) | 35000 µdps/LSB - ATTENTION PAR CALCUL = 30518
 *            | ±2000°/s (±333 tr/min) | 70000 µdps/LSB - ATTENTION PAR CALCUL = 61036.1
 * 
 * author=gael.balduini@gmail.com
 * licence=CC-by-nc-sa
 * url=https://github.com/GBldN/Formation_IMU
 */

  #define f_ech         250                     //Choix d'une fréquence en Hz pour le calcul à intervalle régulier
  #define sensi_acc     488                     //Sensibilité de l'accéléromètre en milli g/LSB   (réglé par défault pour 16 g     calculé en fonction de la pleine échelle)
  #define sensi_gyr   70000                     //Sensibilité du gyromètre       en milli dps/LSB (réglé par défault pour 2000 °/s calculé en fonction de la pleine échelle)
  #define alpha         0.9                     //Choix d'un coefficient pour la fusion par filtre complémentaire
  
  #include "Wire.h"                             //Appel de la bibliothèque de gestion de la communication I2C
  #include "LSM6DS3.h"                          //Appel de la bibliothèque de la centrale
  LSM6DS3 myIMU(I2C_MODE, 0x6A);                //Déclaration de la méthode "MyIMU" pour l'utilisation de la centrale
  #include <MadgwickAHRS.h>                     //Bibliothèque de gestion du timer pour le calcul de la fréquence
  Madgwick filtre_madgwick;

  int16_t  ax_brut, ay_brut, az_brut;           //Variables pour la lecture des valeurs brutes d'accélérations (accéléromètre)
  int16_t  gx_brut, gy_brut, gz_brut;           //Variables pour la lecture des valeurs brutes des vitesses angulaires (gyromètre)
  int32_t  ax_offset, ay_offset, az_offset;     //Variables pour le stockage des valeurs d'offsets de l'accéléromètre
  int32_t  gx_offset, gy_offset, gz_offset;     //Variables pour le stockage des valeurs d'offsets du gyromètre
  float    Ax,Ay,Az,Gx,Gy,Gz;                   //Variables pour le calcul des valeurs réelles
  float    roulis_acc, tangage_acc;             //Variables pour le calcul des angles d'inclinaisons avec l'accéléromètre : ROULIS et TANGAGE
  float    roulis_gyr, tangage_gyr, lacet_gyr;  //Variables pour le calcul des angles d'inclinaisons avec le gyromètre : ROULIS, TANGAGE et LACET
  float    roulis, tangage, lacet;              //Variables pour le calcul des angles d'inclinaisons fusionnés
  
  uint32_t t0;                                  //Variable pour stocker le temps absolu ( fonction millis() débordement après 50 jours environ )

/* #### ROUTINE D'INITIALISATION  (exécutée une seule fois) #### */
void setup() 
{
  Serial.begin(115200);                         //démarrage de la fonction de communication série pour affichage sur le moniteur série
  /* -- Démarrage de la centrale et test de bon fonctionnement -- */
  if (myIMU.begin() != 0) {
      Serial.println("Problème de connexion avec la centrale");
  } else {
      Serial.println("Centrale prête à fonctionner ! ");
  }
  
  filtre_madgwick.begin(f_ech);                 //Lancement de la procédure de la bibliothèque du filtre de magwick
  etalonnage();                                 //Appel de la routine d'étalonnage
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

  /* -- Calcul des angles de ROULIS et TANGAGE à partir de l'accéléromètre -- */
    //CHOIX N°1 LACET autour de Z puis ROULIS autour de X puis TANGAGE autour de Y
  if ( Az >= 0) roulis_acc  = atan2( Ay ,   sqrt( pow(Ax, 2) + pow(Az, 2)) ) * RAD_TO_DEG;
  else          roulis_acc  = atan2( Ay , - sqrt( pow(Ax, 2) + pow(Az, 2)) ) * RAD_TO_DEG;
  
                tangage_acc = atan2( - Ax , Az ) * RAD_TO_DEG;
  
  /* -- Calcul des vitesses angulaires autour des axes de ROULIS, TANGAGE et LACET en fonction de l'inclinaison mesurée par l'accéléromètre  -- */  
  float lacet_vitesse   = ( -Gx * sin(tangage * DEG_TO_RAD) + Gz * cos(tangage * DEG_TO_RAD) ) / cos(roulis * DEG_TO_RAD);
  float roulis_vitesse  = Gx * cos(tangage * DEG_TO_RAD) + Gz * sin(tangage * DEG_TO_RAD);  
  float tangage_vitesse = Gy + ( Gx * sin(tangage * DEG_TO_RAD) - Gz * cos(tangage * DEG_TO_RAD) ) * tan(roulis * DEG_TO_RAD);
  
  /* -- Calcul des angles de ROULIS ET TANGAGE à partir du gyromètre à chaque période (voir explications des calculs) -- */
  uint16_t T_ech = millis() - t0;     //Mesure du temps écoulé depuis le dernier calcul
  
      //Calcul à intervalle régulier si le temps atteint la période T_ech = 1 /f_ech (fréquence d'échantillonnage)
  if ( T_ech >= ( 1000 / f_ech) )     //Calcul à intervalle régulier si le temps atteint la période T_ech = 1 /f_ech
  {
    // Calcul des angles mesurés par le gyromètre par intégration numérique
    lacet_gyr   += lacet_vitesse   * float(T_ech) / 1000;
    roulis_gyr  += roulis_vitesse  * float(T_ech) / 1000;
    tangage_gyr += tangage_vitesse * float(T_ech) / 1000;  
    
    // Modulation de l'angle calculé avec le gyromètre pour qu'il reste entre -180° et +180°
    if (roulis_gyr  <  180) roulis_gyr +=360;
    if (roulis_gyr  >= 180) roulis_gyr -=360;
    if (tangage_gyr <  180) tangage_gyr +=360;
    if (tangage_gyr >= 180) tangage_gyr -=360;
    if (lacet_gyr <  180)   lacet += 360;
    if (lacet_gyr >= 180)   lacet -= 360;
    
    // Calcul des angles fusionnés par le filtre COMPLEMENTAIRE
    //roulis  = alpha * roulis_gyr  + ( 1 - alpha) * roulis_acc;
    //tangage = alpha * tangage_gyr + ( 1 - alpha) * tangage_acc;

    //Calcul des angles avec le filtre de Madgwick
    filtre_madgwick.updateIMU(Gx, Gy, Gz, Ax, Ay, Az);
    roulis  = filtre_madgwick.getRoll();
    tangage = filtre_madgwick.getPitch();
    lacet   = 180 - filtre_madgwick.getYaw();
    
    t0 = millis();  //réinitialisation du temps pour le prochain calcul
  }

  
  /* -- Affichage des valeurs lues par la centrale -- */
  //Serial.print("ACC ( ");    Serial.print(Ax,1); Serial.print(" , "); Serial.print(Ay,1);  Serial.print(" , ");  Serial.print(Az,1); Serial.print(" ) g ");
  //Serial.print(" | GYR ( "); Serial.print(Gx,0); Serial.print(", ");  Serial.print(Gy,0);  Serial.print(" , ");  Serial.print(Gz,0); Serial.print(" ) deg/s");
   
  Serial.print(" | ROULIS Acc = ");  Serial.print(roulis_acc , 0);  Serial.print(" °");
  Serial.print(" ; Gyr = ");         Serial.print(roulis_gyr , 0);  Serial.print(" °");
  Serial.print(" # FUS = ");         Serial.print(roulis , 0);      Serial.print(" °");
  
  Serial.print(" | TANGAGE_Acc = "); Serial.print(tangage_acc , 0); Serial.print(" °"); 
  Serial.print(" ; Gyr = ");         Serial.print(tangage_gyr , 0); Serial.print(" °");
  Serial.print(" # FUS = ");         Serial.print(tangage , 0);     Serial.print(" °");

  Serial.print(" - LACET = ");       Serial.print(lacet , 0);       Serial.print(" °");
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
