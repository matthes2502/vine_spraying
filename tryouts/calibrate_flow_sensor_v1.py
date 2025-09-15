#!/usr/bin/env python3
import time
from gpiozero import Button
import threading

class FlowSensor:
    def __init__(self, gpio_pin=24):
        self.sensor = Button(gpio_pin, pull_up=True)
        self.pulse_count = 0
        self.last_time = time.time()
        self.flow_frequency = 0.0
        self.calibration_factor = 1.0  # wird später berechnet

        self.sensor.when_pressed = self._pulse_callback
        self.timer = threading.Timer(1.0, self._calculate_frequency)
        self.timer.start()
        
    def _pulse_callback(self):
        self.pulse_count += 1
        
    def _calculate_frequency(self):
        current_time = time.time()
        time_diff = current_time - self.last_time
        self.flow_frequency = self.pulse_count / time_diff
        self.pulse_count = 0
        self.last_time = current_time
        self.timer = threading.Timer(0.5, self._calculate_frequency)
        self.timer.start()
        
    def get_flow_rate(self):
        # Basis: Frequenz in Hz → l/min (nur skaliert mit calibration_factor)
        l_min = self.flow_frequency * self.calibration_factor
        return {
            'frequency_hz': round(self.flow_frequency, 2),
            'l_per_minute': round(l_min, 4),
            'l_per_hour': round(l_min * 60, 2),
            'l_per_second': round(l_min / 60, 5),
        }

    def calibrate(self, known_flow_l_min):
        """
        Kalibriert den Faktor:
        known_flow_l_min = bekannter realer Durchfluss in l/min
        """
        if self.flow_frequency > 0:
            self.calibration_factor = known_flow_l_min / self.flow_frequency
            print(f"[CALIBRATION] Faktor gesetzt auf {self.calibration_factor:.5f}")
        else:
            print("[CALIBRATION] Frequenz ist 0 Hz, kann nicht kalibrieren!")

    def stop(self):
        if self.timer:
            self.timer.cancel()


if __name__ == "__main__":
    print("Flow Sensor gestartet...")
    flow = FlowSensor(gpio_pin=24)

    try:
        # Phase 1: Frequenz stabilisieren lassen
        print("Bitte Pumpe auf 1710 PWM laufen lassen...")
        time.sleep(5)  # etwas warten, bis sich Drehzahl stabilisiert

        # Kalibrieren bei bekannter Fördermenge
        flow.calibrate(known_flow_l_min=10.0)
        # calibration factor in mehreren Messungen ca. 0.082

        while True:
            time.sleep(0.5)
            flow_data = flow.get_flow_rate()
            print(f"Frequenz: {flow_data['frequency_hz']} Hz")
            print(f"Flow: {flow_data['l_per_minute']} l/min | "
                  f"{flow_data['l_per_hour']} l/h | "
                  f"{flow_data['l_per_second']} l/s")
            print("-" * 40)

    except KeyboardInterrupt:
        flow.stop()
        print("Fertig.")
