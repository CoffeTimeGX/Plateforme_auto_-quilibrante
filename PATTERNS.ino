#include <AccelStepper.h>
#include <InverseKinematics.h>
#include <MultiStepper.h>
#include <stdint.h>
#include <TouchScreen.h>
#include <math.h>

/**
 * @brief Instance du système cinématique pour le calcul des angles des actionneurs.
 * Configurée avec les paramètres géométriques :
 * - Portée de la base : 2 unités (distance du centre au coin).
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
int targetPos[3];                       // Positions cibles pour les actionneurs
const int ENABLE_PIN = 0;               // Broche d'activation du driver
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
const double PID_P = 4E-4;      // Gain proportionnel
const double PID_I = 2E-6;      // Gain intégral
const double PID_D = 7E-3;      // Gain dérivé
double pidError[2] = {0.0, 0.0}; // Erreurs actuelles (X, Y)
double pidErrorPrior[2];         // Erreurs précédentes
double pidIntegral[2] = {0.0, 0.0}; // Termes intégraux
double pidDerivative[2] = {0.0, 0.0}; // Termes dérivés
double pidOutput[2];             // Sorties PID
long cycleStart, patternTimer;   // Timestamps pour le timing du cycle et du motif

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
 * @brief Boucle de contrôle principale pour l'exécution des motifs.
 * Actuellement, l'exécution est arrêtée ; décommenter les motifs selon les besoins.
 */
void loop() {
  // stabilizeBall(0.0, 0.0); // Maintient la bille à l'origine
  // navigateToTarget(0, 0, 5000); // Se déplace vers la position et maintient
  // linearTrajectory(100, 0, 800, 2); // Trajectoire linéaire
  // triangularTrajectory(3); // Trajectoire triangulaire
  // squareTrajectory(3); // Trajectoire carrée
  // pinballMotion(200, 600); // Motif de flipper
  // ellipticalTrajectory(100, 100, 0, 20, 5); // Trajectoire elliptique
  // sinusoidalMotion(50, 30, 20); // Mouvement sinusoïdal
  // figureEightMotion(200, 0, 10, 5); // Trajectoire en huit
  // executeDemo(); // Exécute tous les motifs séquentiellement
  while (1) {}
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
    for (int i = 0; i < 3; i++) {
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
    for (int i = 0; i < 3; i++) {
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
    for (int i = 0; i < 2; i++) {
      pidErrorPrior[i] = pidError[i];
      pidError[i] = (i == 0) * (X_CENTER - touchData.x - targetX) + (i == 1) * (Y_CENTER - touchData.y - targetY);
      pidIntegral[i] += pidError[i] + pidErrorPrior[i];
      pidDerivative[i] = pidError[i] - pidErrorPrior[i];
      pidDerivative[i] = (isnan(pidDerivative[i]) || isinf(pidDerivative[i])) ? 0.0 : pidDerivative[i];
      pidOutput[i] = PID_P * pidError[i] + PID_I * pidIntegral[i] + PID_D * pidDerivative[i];
      pidOutput[i] = constrain(pidOutput[i], -0.25, 0.25);
    }

    // Calcule les vitesses des actionneurs
    for (int i = 0; i < 3; i++) {
      velocityPrior[i] = velocity[i];
      velocity[i] = (i == NEXUS_X) * actuatorX.currentPosition() + 
                    (i == NEXUS_Y) * actuatorY.currentPosition() + 
                    (i == NEXUS_Z) * actuatorZ.currentPosition();
      velocity[i] = abs(velocity[i] - targetPos[i]) * VELOCITY_GAIN;
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

/**
 * @brief Déplace la bille vers une position cible et maintient pendant une durée.
 * @param xTarget Point de consigne de la coordonnée X.
 * @param yTarget Point de consigne de la coordonnée Y.
 * @param duration Temps de maintien en millisecondes.
 */
void navigateToTarget(int xTarget, int yTarget, int duration) {
  patternTimer = millis();
  while (millis() - patternTimer < duration) {
    stabilizeBall(xTarget, yTarget);
  }
}

/**
 * @brief Déplace la bille selon une trajectoire linéaire.
 * @param xRange Moitié de la longueur de la traversée X (0-200).
 * @param yRange Moitié de la longueur de la traversée Y (0-200).
 * @param pause Délai entre les mouvements en millisecondes (0-1000).
 * @param iterations Nombre de cycles.
 */
void linearTrajectory(double xRange, int yRange, int pause, int iterations) {
  for (int i = 0; i < iterations; i++) {
    patternTimer = millis();
    while (millis() - patternTimer < pause) {
      stabilizeBall(xRange, yRange);
    }
    patternTimer = millis();
    while (millis() - patternTimer < pause) {
      stabilizeBall(-xRange, -yRange);
    }
  }
}

/**
 * @brief Déplace la bille selon une trajectoire triangulaire.
 * @param iterations Nombre de cycles.
 */
void triangularTrajectory(int iterations) {
  double sideLength = 400.0; // Longueur du côté du triangle
  for (int i = 0; i < iterations; i++) {
    for (int j = 0; j < 3; j++) {
      patternTimer = millis();
      while (millis() - patternTimer < 800) {
        stabilizeBall((1 - j) * (sideLength / 2.0), j == 1 ? sideLength * (sqrt(3.0) / 4.0) : -sideLength * (sqrt(3.0) / 4.0));
      }
    }
  }
}

/**
 * @brief Déplace la bille selon une trajectoire carrée.
 * @param iterations Nombre de cycles.
 */
void squareTrajectory(int iterations) {
  int sideLength = 400; // Longueur du côté du carré
  for (int i = 0; i < iterations; i++) {
    for (int j = 0; j < 4; j++) {
      patternTimer = millis();
      while (millis() - patternTimer < 700) {