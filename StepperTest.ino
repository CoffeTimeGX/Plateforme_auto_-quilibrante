#include <AccelStepper.h>
#include <MultiStepper.h>

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
int targetPos[3] = {400, 400, 400}; // Positions cibles pour les actionneurs
const int ENABLE_PIN = 0;           // Broche d'activation du driver

/**
 * @brief Initialise le système des moteurs pas à pas et exécute un mouvement de test.
 * Configure les actionneurs, active les drivers et se déplace vers des positions prédéfinies.
 */
void setup() {
  // Définit la vitesse maximale initiale pour les actionneurs (pas par seconde)
  actuatorX.setMaxSpeed(200.0);
  actuatorY.setMaxSpeed(200.0);
  actuatorZ.setMaxSpeed(200.0); // Correction de l'erreur de frappe : actuatorashi -> actuatorZ

  // Configure le contrôle multi-actionneurs
  actuatorGroup.addStepper(actuatorX);
  actuatorGroup.addStepper(actuatorY);
  actuatorGroup.addStepper(actuatorZ);

  // Initialise la broche d'activation
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Active les drivers
  delay(1000);                   // Pause pour la réinitialisation de la plateforme

  // Exécute le mouvement
  actuatorGroup.moveTo(targetPos); // Calcule les vitesses requises
  actuatorGroup.runSpeedToPosition(); // Bloque jusqu'à ce que les positions cibles soient atteintes
}

/**
 * @brief Boucle principale, actuellement inactive.
 * Espace réservé pour les opérations continues, si nécessaire.
 */
void loop() {
}