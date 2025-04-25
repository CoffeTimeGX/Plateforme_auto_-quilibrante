#include <AccelStepper.h>
#include <InverseKinematics.h>
#include <MultiStepper.h>
#include <stdint.h>
#include <TouchScreen.h>
#include <math.h>

/**
 * @brief Instance du système cinématique pour le calcul des angles des actionneurs.
 * Configuré avec les paramètres géométriques :
 * - Portée de la base : 2 unités (distance du centre à l'extrémité).
 * - Portée de la plateforme : 3.125 unités (distance du centre au coin).
 * - Lien primaire : 1.75 unités (longueur du lien #1).
 * - Lien secondaire : 3.669291339 unités (longueur du lien #2).
 */
CelestialRig kinematicCore(2.0, 3.125, 1.75, 3.669291339);

/**
 * @brief Interface de l'écran tactile pour détecter la position de la bille.
 * Initialisée avec les broches analogiques (A1, A0, A3, A2) et sans décalage de calibration.
 */
TouchScreen touchSensor(A1, A0, A3, A2, 0);

/**
 * @brief Instances des moteurs pas à pas pour le contrôle des actionneurs.
 * Configurées avec le type de driver 1 et les broches de pas/direction respectives.
 */
AccelStepper actuatorX(1, 1, 2);  // Actionneur X (pas : 1, dir : 2)
AccelStepper actuatorY(1, 3, 4);  // Actionneur Y (pas : 3, dir : 4)
AccelStepper actuatorZ(1, 5, 6);  // Actionneur Z (pas : 5, dir : 6)
MultiStepper actuatorGroup;        // Instance du contrôleur multi-pas

/**
 * @brief Variables de contrôle des actionneurs.
 */
int32_t targetPos[3];                   // Positions cibles pour les actionneurs
const uint8_t ENABLE_PIN = 0;           // Broche d'activation du driver
const double BASE_ANGLE = 206.662752199; // Angle initial des actionneurs
double velocity[3] = {0.0, 0.0, 0.0};   // Vitesses actuelles des actionneurs
double velocityPrior[3];                // Vitesses précédentes des actionneurs
const double VELOCITY_GAIN = 20.0;      // Facteur d'amplification de la vitesse

/**
 * @brief Paramètres de calibration de l'écran tactile.
 */
const double X_CENTER = 500.0; // Décalage du centre de l'axe X
const double Y_CENTER = 500.0; // Décalage du centre de l'axe Y

/**
 * @brief Paramètres de contrôle PID pour le positionnement de la bille.
 */
const double PID_P = 9E-9;      // Gain proportionnel (à déterminer)
const double PID_I = 9E-9;      // Gain intégral (à déterminer)
const double PID_D = 9E-9;      // Gain dérivé (à déterminer)
double pidError[2] = {0.0, 0.0}; // Erreurs actuelles (X, Y)
double pidErrorPrior[2];         // Erreurs précédentes
double pidIntegral[2] = {0.0, 0.0}; // Termes intégraux
double pidDerivative[2] = {0.0, 0.0}; // Termes dérivés
double pidOutput[2];             // Sorties PID
uint32_t cycleStart;             // Timestamp pour le timing du cycle

/**
 * @brief État du système et facteurs de conversion.
 */
const double STEP_PER_DEGREE = 3200.0 / 360.0; // Pas par degré (16 micropas)
bool ballPresent = false;                     // Indicateur de présence de la bille

/**
 * @brief Initialise les composants du système et calibre la plateforme à l'origine.
 * Configure la communication série, les moteurs pas à pas et déplace la plateforme à la position initiale.
 */
void setup() {
  Serial.begin(115200);

  // Configure le contrôle multi-pas
  actuatorGroup.addStepper(actuatorX);
  actuatorGroup.addStepper(actuatorY);
  actuatorGroup.addStepper(actuatorZ);

  // Initialise la broche d'activation
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // Désactive les drivers initialement
  delay(1000);                    // Pause pour la réinitialisation de la plateforme
  digitalWrite(ENABLE_PIN, LOW);  // Active les drivers

  // Déplace vers la position d'origine
  repositionPlatform(4.25, 0.0, 0.0);
  actuatorGroup.runSpeedToPosition(); // Attend que la position d'origine soit atteinte
}

/**
 * @brief Boucle de contrôle principale pour l'équilibrage de la bille.
 * Exécute le contrôle PID avec les points de consigne spécifiés.
 */
void loop() {
  stabilizeBall(0.0, 0.0); // Maintient la bille à l'origine (X=0, Y=0)
}

/**
 * @brief Positionne la plateforme en fonction des paramètres spatiaux.
 * @param zPos Position verticale de la plateforme.
 * @param xVec Composante X de l'orientation de la plateforme.
 * @param yVec Composante Y de l'orientation de la plateforme.
 */
void repositionPlatform(double zPos, double xVec, double yVec) {
  if (ballPresent) {
    // Calcule les positions des actionneurs
    for (uint8_t i = 0; i < 3; ++i) {
      targetPos[i] = round((BASE_ANGLE - kinematicCore.computeAngle(i, zPos, xVec, yVec)) * STEP_PER_DEGREE);
    }

    // Configure les paramètres des actionneurs
    actuatorX.setMaxSpeed(velocity[NEXUS_X]);
    actuatorY.setMaxSpeed(velocity[NEXUS_Y]);
    actuatorZ.setMaxSpeed(velocity[NEXUS_Z]);

    actuatorX.setAcceleration(velocity[NEXUS_X] * 30.0);
    actuatorY.setAcceleration(velocity[NEXUS_Y] * 30.0);
    actuatorZ.setAcceleration(velocity[NEXUS_Z] * 30.0);

    actuatorX.moveTo(targetPos[NEXUS_X]);
    actuatorY.moveTo(targetPos[NEXUS_Y]);
    actuatorZ.moveTo(targetPos[NEXUS_Z]);

    // Exécute les mouvements des actionneurs
    actuatorX.run();
    actuatorY.run();
    actuatorZ.run();
  } else {
    // Calcule les positions par défaut (sans inclinaison)
    for (uint8_t i = 0; i < 3; ++i) {
      targetPos[i] = round((BASE_ANGLE - kinematicCore.computeAngle(i, zPos, 0.0, 0.0)) * STEP_PER_DEGREE);
    }

    // Définit une vitesse fixe
    actuatorX.setMaxSpeed(800.0);
    actuatorY.setMaxSpeed(800.0);
    actuatorZ.setMaxSpeed(800.0);

    // Déplace les actionneurs
    actuatorGroup.moveTo(targetPos);
    actuatorGroup.run();
  }
}

/**
 * @brief Exécute le contrôle PID pour positionner la bille.
 * @param targetX Coordonnée X désirée de la bille.
 * @param targetY Coordonnée Y désirée de la bille.
 */
void stabilizeBall(double targetX, double targetY) {
  TSPoint touchData = touchSensor.getPoint(); // Lit l'écran tactile

  if (touchData.x != 0) {
    ballPresent = true;

    // Calcule les termes PID
    for (uint8_t i = 0; i < 2; ++i) {
      pidErrorPrior[i] = pidError[i];
      pidError[i] = (i == 0) * (X_CENTER - touchData.x - targetX) +(i ==1) *(Y_CENTER - touchData.y - targetY);
      pidIntegral[i] += pidError[i] + pidErrorPrior[i];
      pidDerivative[i] = pidError[i] - pidErrorPrior[i];
      pidDerivative[i] = (isnan(pidDerivative[i]) || isinf(pidDerivative[i])) ? 0.0 : pidDerivative[i];
      pidOutput[i] = PID_P * pidError[i] + PID_I * pidIntegral[i] + PID_D * pidDerivative[i];
      pidOutput[i] = constrain(pidOutput[i], -0.25, 0.25);
    }

    // Calcule les vitesses des actionneurs
    for (uint8_t i = 0; i < 3; ++i) {
      velocityPrior[i] = velocity[i];
      int32_t currentPos = (i == NEXUS_X) ? actuatorX.currentPosition() :
                           (i == NEXUS_Y) ? actuatorY.currentPosition() :
                           actuatorZ.currentPosition();
      velocity[i] = abs(currentPos - targetPos[i]) * VELOCITY_GAIN;
      velocity[i] = constrain(velocity[i], velocityPrior[i] - 200.0, velocityPrior[i] + 200.0);
      velocity[i] = constrain(velocity[i], 0.0, 1000.0);
    }

    // Sortie pour le diagnostic
    Serial.println((String) "X Output = " + pidOutput[0] + "   Y Output = " + pidOutput[1] + "   Velocity X: " + velocity[NEXUS_X]);
  } else {
    delay(10);
    touchData = touchSensor.getPoint();
    if (touchData.x == 0) {
      ballPresent = false;
    }
  }

  // Maintient le mouvement de la plateforme pour un cycle de 20ms
  cycleStart = millis();
  while (millis() - cycleStart < 20) {
    repositionPlatform(4.25, -pidOutput[0], -pidOutput[1]);
  }
}