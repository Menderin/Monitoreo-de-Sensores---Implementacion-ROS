#!/usr/bin/env python3
"""
Herramienta de Calibración pH - Captura medianas de voltaje raw
Modo interactivo para calibrar sensor de pH con soluciones buffer

Flujo:
1. Espacio para iniciar
2. 10 muestras de estabilización (descartadas)
3. 30 muestras reales
4. Muestra mediana
5. Espacio para repetir, Enter/Escape para salir
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys
import termios
import tty
import statistics
from datetime import datetime
import time


class CalibrationNode(Node):
    """
    Nodo para calibración de pH usando medianas
    """

    def __init__(self):
        super().__init__('ph_calibration')
        
        # Variables para almacenar datos
        self.voltage_samples = []
        self.ph_samples = []
        self.last_voltage = None
        self.last_ph = None
        self.collecting = False
        self.warmup_count = 0
        self.sample_count = 0
        self.last_sample_time = 0  # Para evitar duplicados
        
        # Configuración
        self.WARMUP_SAMPLES = 10   # Muestras de estabilización
        self.REAL_SAMPLES = 30     # Muestras para mediana
        self.MIN_SAMPLE_INTERVAL = 0.3  # Mínimo 300ms entre muestras
        
        # Crear suscriptores
        self.voltage_sub = self.create_subscription(
            Float32, 'voltage_raw_ph', self.voltage_callback, 10)
        self.ph_sub = self.create_subscription(
            Float32, 'ph', self.ph_callback, 10)
        
    def voltage_callback(self, msg):
        self.last_voltage = msg.data
        
        # Solo procesar si estamos recolectando y ha pasado suficiente tiempo
        current_time = time.time()
        if (self.collecting and 
            self.last_ph is not None and
            (current_time - self.last_sample_time) >= self.MIN_SAMPLE_INTERVAL):
            
            self.last_sample_time = current_time
            self._process_sample()
    
    def ph_callback(self, msg):
        self.last_ph = msg.data
    
    def _process_sample(self):
        """Procesa una muestra recibida"""
        if self.warmup_count < self.WARMUP_SAMPLES:
            # Fase de estabilización
            self.warmup_count += 1
            print(f"\r  🔄 Estabilizando... {self.warmup_count}/{self.WARMUP_SAMPLES}  "
                  f"[V: {self.last_voltage:.1f} mV]", end="", flush=True)
        elif self.sample_count < self.REAL_SAMPLES:
            # Fase de muestreo real
            self.voltage_samples.append(self.last_voltage)
            self.ph_samples.append(self.last_ph)
            self.sample_count += 1
            print(f"\r  📊 Muestreando... {self.sample_count}/{self.REAL_SAMPLES} | "
                  f"V: {self.last_voltage:.1f} mV | pH: {self.last_ph:.2f}    ", end="", flush=True)
    
    def start_collection(self):
        """Inicia la recolección de muestras"""
        self.voltage_samples = []
        self.ph_samples = []
        self.warmup_count = 0
        self.sample_count = 0
        self.last_sample_time = 0
        self.collecting = True
    
    def stop_collection(self):
        """Detiene la recolección"""
        self.collecting = False
    
    def is_warmup_complete(self):
        """Verifica si terminó la estabilización"""
        return self.warmup_count >= self.WARMUP_SAMPLES
    
    def is_complete(self):
        """Verifica si se completaron todas las muestras"""
        return self.sample_count >= self.REAL_SAMPLES
    
    def get_median_results(self):
        """Calcula y retorna las medianas"""
        if len(self.voltage_samples) < 1:
            return None, None
        
        median_voltage = statistics.median(self.voltage_samples)
        median_ph = statistics.median(self.ph_samples)
        return median_voltage, median_ph


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


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    
    print("\n" + "="*70)
    print("  🧪 HERRAMIENTA DE CALIBRACIÓN pH")
    print("  Calcula medianas de voltaje raw para calibración")
    print("="*70)
    print("\n  📋 Instrucciones:")
    print("     • Presiona ESPACIO para tomar medición")
    print("     • Se tomarán 10 muestras de estabilización + 30 reales")
    print("     • Presiona ENTER o ESCAPE para salir")
    print("\n" + "-"*70)
    
    results_log = []
    running = True
    
    try:
        while running:
            print("\n\n  👉 Presiona [ESPACIO] para iniciar, [ENTER/ESC] para salir...")
            
            key = get_key()
            
            if key in ['\r', '\n', 'escape']:
                running = False
                break
            
            if key == ' ':
                print("\n")
                print("  ════════════════════════════════════════════════════")
                print("  📊 INICIANDO CAPTURA DE MUESTRAS")
                print("  ════════════════════════════════════════════════════")
                print("  Fase 1: Estabilización (10 muestras)...")
                
                # Iniciar recolección
                node.start_collection()
                
                # Fase 1: Esperar estabilización
                while not node.is_warmup_complete():
                    rclpy.spin_once(node, timeout_sec=0.1)
                
                print("\n  Fase 2: Muestreo real (30 muestras)...")
                
                # Fase 2: Muestras reales
                while not node.is_complete():
                    rclpy.spin_once(node, timeout_sec=0.1)
                
                node.stop_collection()
                
                # Calcular medianas
                median_v, median_ph = node.get_median_results()
                
                if median_v is not None:
                    print("\n\n")
                    print("  ╔══════════════════════════════════════════════════╗")
                    print("  ║            📊 RESULTADO DE MEDICIÓN              ║")
                    print("  ╠══════════════════════════════════════════════════╣")
                    print(f"  ║   Voltaje Raw (mediana): {median_v:8.2f} mV           ║")
                    print(f"  ║   pH calculado (mediana): {median_ph:7.3f}               ║")
                    print("  ╚══════════════════════════════════════════════════╝")
                    
                    # Guardar resultado
                    timestamp = datetime.now().strftime('%H:%M:%S')
                    results_log.append((timestamp, median_v, median_ph))
                    
                    # Mostrar tabla de resultados acumulados
                    if len(results_log) > 1:
                        print("\n  📋 Historial de mediciones:")
                        print("     #   Hora      Voltaje (mV)    pH")
                        print("    ─── ────────  ────────────  ──────")
                        for i, (ts, v, ph) in enumerate(results_log, 1):
                            print(f"    {i:2d}  {ts}     {v:8.2f}       {ph:.3f}")
                else:
                    print("\n  ⚠️  Error: No se recibieron datos suficientes")
    
    except KeyboardInterrupt:
        pass
    
    # Resumen final
    print("\n\n" + "="*70)
    print("  📊 RESUMEN FINAL DE CALIBRACIÓN")
    print("="*70)
    
    if results_log:
        print("\n  Mediciones realizadas:")
        print("  ─────────────────────────────────────────────────")
        print("    #   Hora       Voltaje (mV)    pH (calculado)")
        print("  ─────────────────────────────────────────────────")
        for i, (ts, v, ph) in enumerate(results_log, 1):
            print(f"   {i:2d}   {ts}      {v:8.2f}          {ph:.3f}")
        print("  ─────────────────────────────────────────────────")
        
        print("\n  💡 Para calcular nueva regresión lineal:")
        print("     Usa estos voltajes con los pH REALES de tus soluciones buffer")
        print("     Fórmula: pH = (voltage_mV × SLOPE) + INTERCEPT")
    else:
        print("\n  ⚠️  No se realizaron mediciones")
    
    print("\n" + "="*70)
    print("  🛑 Programa terminado")
    print("="*70 + "\n")
    
    node.destroy_node()
    try:
        rclpy.shutdown()
    except:
        pass


if __name__ == '__main__':
    main()
