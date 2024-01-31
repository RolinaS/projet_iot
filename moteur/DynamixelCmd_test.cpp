// Envoyer une commande au format suivant vers un OpenCM9.04 programmé avec DynamixelCmd.ino :
//    numéro_de_moteur_0_ou_1,position_0_a_1023,vitesse_optionnelle_0_a_1023_0_si_non_présent
// Si la vitesse n'est pas précisée, elle garde la précédente vitesse précisée
// Attention : j'ai activé une limitation de la position (+-20degrés donc position entre 512-70=442 et 512+70=582)

#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>

// Définir le nom du port série et la vitesse de communication
#define PORT_NAME "/dev/ttyACM0"
#define BAUD_RATE "1000000"

int main() {
    // Créer un objet ofstream pour écrire sur le port série
    std::ofstream serial;

    // Définir les commandes de pilotage des moteurs
    std::string commande1 = "0,512,0\n1,512,0\n";
    std::string commande2 = "0,0,10\n1,0,10\n";
    std::string commande3 = "0,1023,20\n1,1023,20\n";

    // Ouvrir le port série
    serial.open(PORT_NAME);
    if (serial.is_open()) {
        std::cout << "Succeeded to open the serial port!\n";
    } else {
        std::cout << "Failed to open the serial port!\n";
        return 0;
    }

    // Configurer la vitesse de communication
    serial << "stty -F " << PORT_NAME << " " << BAUD_RATE << "\n";

    // Envoyer les commandes de pilotage
    serial << commande1;
    serial.flush();

    std::this_thread::sleep_for(std::chrono::seconds(3));

    serial << commande2;
    serial.flush();

    std::this_thread::sleep_for(std::chrono::seconds(3));

    serial << commande3;
    serial.flush();

    std::this_thread::sleep_for(std::chrono::seconds(3));

    serial << commande1;
    serial.flush();

    // Afficher le statut de réussite
    std::cout << "Succeeded to send the command!\n";

    // Fermer le port série
    serial.close();
}