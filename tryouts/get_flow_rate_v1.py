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
        self.last_time = time.time()
        self.flow_frequency = 0.0

        # calibration factor from different code
        self.calibration_factor = 0.082
        
        # Sensor callback registrieren
        self.sensor.when_pressed = self._pulse_callback
        
        # Timer für Frequenz-Berechnung
        self.timer = threading.Timer(1.0, self._calculate_frequency)
        self.timer.start()
        
    def _pulse_callback(self):
        """Callback für jeden Puls vom Sensor"""
        self.pulse_count += 1
        
    def _calculate_frequency(self):
        """Berechnet Frequenz alle 1 Sekunde"""
        current_time = time.time()
        time_diff = current_time - self.last_time
        
        # Frequenz in Hz
        self.flow_frequency = self.pulse_count / time_diff
        
        # Nächsten Timer starten
        self.timer = threading.Timer(0.5, self._calculate_frequency)
        self.timer.start()

        # Reset für nächste Messung
        self.pulse_count = 0
        self.last_time = current_time
        
    def get_flow_rate(self):
        """
        Berechnet Volumenstrom basierend auf Frequenz
        Returns: dict mit l/h, l/min, l/s
        """
        # YF-B2 Formel: l/hour = (frequency * 60 / 7.5)
        # l_hour = (self.flow_frequency * 60) / 7.5
        # l_min = l_hour / 60
        # l_s = l_min / 60

        l_min = self.flow_frequency * self.calibration_factor
        
        return {
            'frequency_hz': round(self.flow_frequency, 2),
            'l_per_hour': round(l_min*60, 3),
            'l_per_minute': round(l_min, 4),
            'l_per_second': round(l_min/60, 5),
            'pulse_count_last_second': self.pulse_count
        }
    
    def stop(self):
        """Stoppt den Timer"""
        if self.timer:
            self.timer.cancel()

# Test/Demo
if __name__ == "__main__":
    print("Flow Sensor gestartet...")
    
    # Sensor initialisieren (GPIO 24)
    flow = FlowSensor(gpio_pin=24)
    
    try:
        while True:
            # Alle 0.5 Sekunden Werte ausgeben
            time.sleep(0.5)
            
            flow_data = flow.get_flow_rate()
            print(f"Frequenz: {flow_data['frequency_hz']} Hz")
            print(f"Pumpendrehzahl: {flow_data['l_per_minute']*140} U/min")
            print(f"Pumpendrehzahl: {flow_data['l_per_minute']*140/60} U/s")
            print(f"Flow: {flow_data['l_per_hour']} l/h | "
                  f"{flow_data['l_per_minute']} l/min | "
                  f"{flow_data['l_per_second']} l/s")
            print(f"Pulse Count: {flow_data['pulse_count_last_second']}")
            print("-" * 48)
            
    except KeyboardInterrupt:
        print("Sensor wird gestoppt...")
        flow.stop()
        print("Fertig.")