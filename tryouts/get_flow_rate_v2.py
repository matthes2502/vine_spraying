#!/usr/bin/env python3

import time
from gpiozero import Button
import threading

class FlowSensor:
    def __init__(self, gpio_pin=24):
        """
        Flow Sensor mit YF-B2
        gpio_pin: GPIO Pin für den Sensor
        """
        self.sensor = Button(gpio_pin, pull_up=True)
        self.pulse_count = 0
        self.total_pulses = 0
        self.flow_frequency = 0.0
        self.lock = threading.Lock()  # Thread-sicher
        
        # Calibration factor
        self.calibration_factor = 0.082
        
        # Sensor callback registrieren
        self.sensor.when_pressed = self._pulse_callback
        
        # Timer für Frequenz-Berechnung (alle 1 Sekunde)
        self.timer_running = True
        self.timer_thread = threading.Thread(target=self._frequency_calculator)
        self.timer_thread.daemon = True
        self.timer_thread.start()
        
    def _pulse_callback(self):
        """Callback für jeden Puls vom Sensor"""
        with self.lock:
            self.pulse_count += 1
            self.total_pulses += 1
        
    def _frequency_calculator(self):
        """Berechnet Frequenz alle 1 Sekunde in separatem Thread"""
        while self.timer_running:
            time.sleep(1.0)  # Genau 1 Sekunde warten
            
            with self.lock:
                # Frequenz = Pulse pro Sekunde
                self.flow_frequency = self.pulse_count
                self.pulse_count = 0  # Reset für nächste Sekunde
        
    def get_flow_rate(self):
        """
        Berechnet Volumenstrom basierend auf Frequenz
        Returns: dict mit l/h, l/min, l/s
        """
        with self.lock:
            current_frequency = self.flow_frequency
            current_total = self.total_pulses
        
        # Volumenstrom berechnen
        l_min = current_frequency * self.calibration_factor
        
        return {
            'frequency_hz': round(current_frequency, 2),
            'l_per_hour': round(l_min * 60, 3),
            'l_per_minute': round(l_min, 4),
            'l_per_second': round(l_min / 60, 5),
            'pulses_last_second': current_frequency,
            'total_pulses': current_total
        }
    
    def reset_total(self):
        """Setzt den Gesamtzähler zurück"""
        with self.lock:
            self.total_pulses = 0
    
    def stop(self):
        """Stoppt den Timer Thread"""
        self.timer_running = False
        if hasattr(self, 'timer_thread'):
            self.timer_thread.join(timeout=2)

# Test/Demo
if __name__ == "__main__":
    print("Flow Sensor gestartet...")
    print("Drücke Ctrl+C zum Beenden")
    
    # Sensor initialisieren (GPIO 24)
    flow = FlowSensor(gpio_pin=24)
    
    try:
        start_time = time.time()
        
        while True:
            # Alle 0.5 Sekunden Werte ausgeben
            time.sleep(0.5)
            
            flow_data = flow.get_flow_rate()
            runtime = time.time() - start_time
            
            print(f"\n=== Laufzeit: {runtime:.1f}s ===")
            print(f"Frequenz: {flow_data['frequency_hz']} Hz")
            print(f"Pulse letzte Sekunde: {flow_data['pulses_last_second']}")
            print(f"Gesamte Pulse: {flow_data['total_pulses']}")
            
            if flow_data['frequency_hz'] > 0:
                print(f"Flow: {flow_data['l_per_hour']} l/h | "
                      f"{flow_data['l_per_minute']} l/min | "
                      f"{flow_data['l_per_second']} l/s")
                
                # Pumpendrehzahl (falls das relevant ist)
                rpm = flow_data['l_per_minute'] * 140
                print(f"Pumpen-RPM: {rpm:.1f} U/min | {rpm/60:.1f} U/s")
            else:
                print("Kein Flow erkannt")
            
            print("-" * 48)
            
    except KeyboardInterrupt:
        print("\nSensor wird gestoppt...")
        flow.stop()
        print("Fertig.")