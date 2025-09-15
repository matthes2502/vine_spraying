#include <iostream>
#include <wiringPi.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <iomanip>

// Globale Variablen für ISR
volatile std::atomic<int> pulse_count(0);
std::chrono::steady_clock::time_point last_measurement;
double flow_frequency = 0.0;

// Interrupt Service Routine
void pulse_interrupt() {
    pulse_count++;
}

class FlowSensor {
private:
    int gpio_pin;
    double calibration_factor;
    
public:
    FlowSensor(int pin_WPI = 10, double cal_factor = 7.5) 
        : gpio_pin(pin_WPI), calibration_factor(cal_factor) {
        
        // WiringPi initialisieren
        if (wiringPiSetupPinType(WPI_PIN_BCM) == -1) {
            std::cerr << "Fehler: WiringPi Setup fehlgeschlagen!" << std::endl;
            exit(1);
        }
        
        // Pin als Input konfigurieren
        pinMode(gpio_pin, INPUT);
        pullUpDnControl(gpio_pin, PUD_UP);  // Pull-up aktivieren
        
        // Interrupt registrieren - RISING edge für bessere Erkennung
        if (wiringPiISR(gpio_pin, INT_EDGE_RISING, &pulse_interrupt) < 0) {
            std::cerr << "Fehler: ISR Setup fehlgeschlagen auf GPIO " << gpio_pin << std::endl;
            exit(1);
        }
        
        // Zeitmessung initialisieren
        last_measurement = std::chrono::steady_clock::now();
        pulse_count = 0;
        
        std::cout << "Flow Sensor initialisiert auf GPIO " << gpio_pin << std::endl;
        std::cout << "Kalibrierungsfaktor: " << calibration_factor << std::endl;
    }
    
    void calculate_frequency() {
        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_measurement);
        double time_diff_seconds = duration.count() / 1000000.0;
        
        // Frequenz berechnen (Hz)
        int current_pulses = pulse_count.exchange(0);  // Atomarer read-and-reset
        flow_frequency = current_pulses / time_diff_seconds;
        
        // Zeit für nächste Messung
        last_measurement = current_time;
    }
    
    struct FlowData {
        double frequency_hz;
        double l_per_hour;
        double l_per_minute;
        double l_per_second;
        int pulses_counted;
    };
    
    FlowData get_flow_rate() {
        FlowData data;
        
        data.frequency_hz = flow_frequency;
        
        // YF-B2 Berechnung: l/hour = (frequency * 60 / calibration_factor)
        data.l_per_hour = (flow_frequency * 60.0) / calibration_factor;
        data.l_per_minute = data.l_per_hour / 60.0;
        data.l_per_second = data.l_per_minute / 60.0;
        data.pulses_counted = pulse_count.load();
        
        return data;
    }
    
    void print_values() {
        FlowData data = get_flow_rate();
        
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Frequenz: " << data.frequency_hz << " Hz" << std::endl;
        std::cout << "Flow: " << data.l_per_hour << " l/h | " 
                  << std::setprecision(4) << data.l_per_minute << " l/min | "
                  << std::setprecision(5) << data.l_per_second << " l/s" << std::endl;
        std::cout << "Pulse (aktuell): " << data.pulses_counted << std::endl;
        std::cout << "----------------------------------------" << std::endl;
    }
};

int main() {
    std::cout << "Flow Sensor Test mit WiringPi ISR gestartet..." << std::endl;
    std::cout << "Drücke Ctrl+C zum Beenden" << std::endl;
    
    // Sensor erstellen (GPIO 18, Kalibrierung 7.5)
    FlowSensor sensor(23, 7.5);
    
    try {
        while (true) {
            // Alle 1 Sekunde Frequenz neu berechnen
            std::this_thread::sleep_for(std::chrono::seconds(1));
            sensor.calculate_frequency();
            
            // Alle 2 Sekunden Werte ausgeben
            static int counter = 0;
            if (++counter >= 2) {
                sensor.print_values();
                counter = 0;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Fehler: " << e.what() << std::endl;
    }
    
    std::cout << "Sensor beendet." << std::endl;
    return 0;
}