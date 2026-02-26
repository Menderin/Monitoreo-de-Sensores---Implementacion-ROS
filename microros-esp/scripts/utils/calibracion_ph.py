#!/usr/bin/env python3
"""
Herramienta de Calibración pH - Captura medianas de voltaje raw
Modo interactivo para calibrar sensor de pH con soluciones buffer

Flujo:
1. Espacio para iniciar medición (sensor en buffer conocido)
2. 10 muestras de estabilización (descartadas)
3. 30 muestras reales → calcula mediana de voltaje
4. El usuario ingresa el pH real de esa solución buffer
5. Repetir para cada buffer (mínimo 2, recomendado 3)
6. Al salir: regresión lineal con numpy → genera valores para config.h

Topic: /sensor_data (Float32MultiArray)
  [0] temperatura
  [1] pH calculado (con calibración anterior)
  [2] voltage_raw_ph  ← lo que usamos para la nueva regresión
  [3] mac_part1
  [4] mac_part2
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys
import termios
import tty
import statistics
import numpy as np
from datetime import datetime
import time


class CalibrationNode(Node):
    """
    Nodo para calibración de pH usando medianas del topic sensor_data
    """

    def __init__(self):
        super().__init__('ph_calibration')

        # Variables para almacenar datos
        self.voltage_samples = []
        self.collecting = False
        self.warmup_count = 0
        self.sample_count = 0
        self.last_sample_time = 0
        self.last_voltage = None

        # Configuración
        self.WARMUP_SAMPLES = 10        # Muestras descartadas (estabilización)
        self.REAL_SAMPLES = 30          # Muestras para calcular mediana
        self.MIN_SAMPLE_INTERVAL = 0.3  # 300ms mínimo entre muestras

        # Suscriptor único al topic real del ESP32
        self.sensor_sub = self.create_subscription(
            Float32MultiArray,
            'sensor_data',
            self.sensor_callback,
            10)

    def sensor_callback(self, msg):
        """Callback del topic sensor_data [temp, pH, voltage_raw, mac1, mac2]"""
        if len(msg.data) < 3:
            return

        voltage = msg.data[2]   # índice 2 = voltage_raw_ph en mV
        self.last_voltage = voltage

        current_time = time.time()
        if (self.collecting and
                (current_time - self.last_sample_time) >= self.MIN_SAMPLE_INTERVAL):
            self.last_sample_time = current_time
            self._process_sample(voltage)

    def _process_sample(self, voltage):
        """Procesa una muestra recibida"""
        if self.warmup_count < self.WARMUP_SAMPLES:
            self.warmup_count += 1
            print(f"\r  🔄 Estabilizando... {self.warmup_count}/{self.WARMUP_SAMPLES}"
                  f"  [V: {voltage:.1f} mV]", end="", flush=True)
        elif self.sample_count < self.REAL_SAMPLES:
            self.voltage_samples.append(voltage)
            self.sample_count += 1
            print(f"\r  📊 Muestreando... {self.sample_count}/{self.REAL_SAMPLES}"
                  f" | V: {voltage:.1f} mV", end="", flush=True)

    def start_collection(self):
        self.voltage_samples = []
        self.warmup_count = 0
        self.sample_count = 0
        self.last_sample_time = 0
        self.collecting = True

    def stop_collection(self):
        self.collecting = False

    def is_warmup_complete(self):
        return self.warmup_count >= self.WARMUP_SAMPLES

    def is_complete(self):
        return self.sample_count >= self.REAL_SAMPLES

    def get_median_voltage(self):
        if not self.voltage_samples:
            return None
        return statistics.median(self.voltage_samples)


def get_key():
    """Lee una tecla del teclado sin esperar Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            return 'escape'
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def ask_ph_value(median_v):
    """Pide al usuario el pH real de la solución buffer (restaura terminal normal)"""
    print(f"\n\n  El voltaje medido fue: {median_v:.2f} mV")
    print("  Ingresa el pH REAL de esta solución buffer")
    print("  (ej: 4.01, 6.86, 9.18) y presiona Enter: ", end="", flush=True)
    try:
        val = input()
        return float(val.strip())
    except (ValueError, EOFError):
        return None


def compute_regression(calibration_points):
    """
    Calcula regresión lineal: pH = slope * voltage_mV + intercept
    calibration_points: lista de (voltage_mV, ph_real)
    """
    voltages = np.array([p[0] for p in calibration_points])
    ph_values = np.array([p[1] for p in calibration_points])

    # Regresión lineal grado 1: [slope, intercept]
    coeffs = np.polyfit(voltages, ph_values, 1)
    slope = coeffs[0]
    intercept = coeffs[1]

    # Calcular R² para evaluar la calidad del ajuste
    ph_pred = np.polyval(coeffs, voltages)
    ss_res = np.sum((ph_values - ph_pred) ** 2)
    ss_tot = np.sum((ph_values - np.mean(ph_values)) ** 2)
    r_squared = 1 - (ss_res / ss_tot) if ss_tot != 0 else 1.0

    return slope, intercept, r_squared


def print_regression_results(calibration_points, slope, intercept, r_squared):
    """Muestra resultados de la regresión y el bloque listo para config.h"""

    date_str = datetime.now().strftime('%Y-%m-%d')
    buffers_str = ", ".join(
        [f"pH {ph:.2f}→{v:.0f}mV" for v, ph in calibration_points]
    )

    print("\n\n" + "="*70)
    print("  📐 REGRESIÓN LINEAL - RESULTADOS")
    print("="*70)
    print(f"\n  Puntos de calibración usados: {len(calibration_points)}")
    print(f"  {'Voltaje (mV)':>15}  {'pH buffer':>10}  {'pH predicho':>12}  {'Error':>8}")
    print("  " + "─"*56)

    for v, ph_real in calibration_points:
        ph_pred = slope * v + intercept
        error = ph_real - ph_pred
        print(f"  {v:>15.2f}  {ph_real:>10.3f}  {ph_pred:>12.3f}  {error:>+8.4f}")

    print("  " + "─"*56)
    print(f"\n  slope     = {slope:.6f}")
    print(f"  intercept = {intercept:.6f}")
    print(f"  R²        = {r_squared:.6f}", end="")

    if r_squared >= 0.999:
        print("  ✅ Excelente ajuste")
    elif r_squared >= 0.995:
        print("  ✅ Buen ajuste")
    elif r_squared >= 0.98:
        print("  ⚠️  Ajuste aceptable (revisar puntos)")
    else:
        print("  ❌ Ajuste pobre — revisa las mediciones")

    # Bloque listo para copiar en config.h
    print("\n\n  ┌─────────────────────────────────────────────────────────────┐")
    print("  │  📋 COPIAR EN config.h                                       │")
    print("  ├─────────────────────────────────────────────────────────────┤")
    print(f"  │  // pH: calibrado {date_str:<43}│")
    print(f"  │  // Buffers: {buffers_str:<49}│")
    print(f"  │  #define PH_SLOPE       {slope:<38.6f}│")
    print(f"  │  #define PH_INTERCEPT   {intercept:<38.6f}│")
    print("  └─────────────────────────────────────────────────────────────┘")


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()

    print("\n" + "="*70)
    print("  🧪 HERRAMIENTA DE CALIBRACIÓN pH")
    print("  Topic: /sensor_data[2] = voltage_raw_ph")
    print("="*70)
    print("\n  📋 Instrucciones:")
    print("     1. Sumerge el sensor en la solución buffer")
    print("     2. Presiona ESPACIO para iniciar captura")
    print("     3. Ingresa el pH real de esa solución cuando se pida")
    print("     4. Repite para cada buffer (mínimo 2, ideal 3)")
    print("     5. Presiona ENTER o ESCAPE para calcular la regresión")
    print("\n" + "-"*70)

    # Lista de puntos de calibración: (voltage_mV, ph_real)
    calibration_points = []
    running = True

    try:
        while running:
            n = len(calibration_points)
            print(f"\n\n  👉 [{n} punto(s) capturado(s)]  "
                  f"[ESPACIO]=nueva medición  [ENTER/ESC]=calcular y salir")

            key = get_key()

            if key in ['\r', '\n', 'escape']:
                running = False
                break

            if key == ' ':
                print("\n")
                print("  ════════════════════════════════════════")
                print("  📊 CAPTURANDO MUESTRAS")
                print("  ════════════════════════════════════════")
                print("  Fase 1: Estabilización (10 muestras)...")

                node.start_collection()

                # Fase 1: estabilización
                while not node.is_warmup_complete():
                    rclpy.spin_once(node, timeout_sec=0.1)

                print("\n  Fase 2: Muestreo real (30 muestras)...")

                # Fase 2: muestras reales
                while not node.is_complete():
                    rclpy.spin_once(node, timeout_sec=0.1)

                node.stop_collection()

                median_v = node.get_median_voltage()

                if median_v is None:
                    print("\n\n  ⚠️  No se recibieron datos. "
                          "¿Está el ESP32 publicando en /sensor_data?")
                    continue

                print(f"\n\n  ╔══════════════════════════════════════════╗")
                print(f"  ║   Voltaje mediano: {median_v:8.2f} mV          ║")
                print(f"  ╚══════════════════════════════════════════╝")

                # Pedir pH real al usuario
                ph_real = ask_ph_value(median_v)
                if ph_real is None:
                    print("  ⚠️  pH inválido, medición descartada")
                    continue

                if not (0.0 <= ph_real <= 14.0):
                    print(f"  ⚠️  pH {ph_real} fuera de rango (0-14), medición descartada")
                    continue

                calibration_points.append((median_v, ph_real))
                print(f"\n  ✅ Punto guardado: {median_v:.2f} mV → pH {ph_real:.2f}")

                # Mostrar tabla acumulada
                if len(calibration_points) > 1:
                    print("\n  📋 Puntos de calibración acumulados:")
                    print("     #   Voltaje (mV)    pH real")
                    print("    ─── ────────────  ──────────")
                    for i, (v, ph) in enumerate(calibration_points, 1):
                        print(f"    {i:2d}   {v:10.2f}       {ph:.2f}")

    except KeyboardInterrupt:
        pass

    # ========================================
    # REGRESIÓN LINEAL FINAL
    # ========================================
    print("\n\n" + "="*70)
    print("  📊 RESUMEN FINAL DE CALIBRACIÓN")
    print("="*70)

    if len(calibration_points) < 2:
        print(f"\n  ⚠️  Se necesitan al menos 2 puntos para la regresión.")
        print(f"     Puntos capturados: {len(calibration_points)}")
    else:
        slope, intercept, r_squared = compute_regression(calibration_points)
        print_regression_results(calibration_points, slope, intercept, r_squared)

    print("\n" + "="*70)
    print("  🛑 Programa terminado")
    print("="*70 + "\n")

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
