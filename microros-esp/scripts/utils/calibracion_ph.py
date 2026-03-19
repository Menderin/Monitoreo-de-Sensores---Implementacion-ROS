#!/usr/bin/env python3
"""
Herramienta de Calibración pH - Captura medianas de voltaje raw
Modo interactivo para calibrar sensor de pH con soluciones buffer

Flujo:
 1. Conecta a MongoDB y lista dispositivos registrados (por fecha desc)
 2. Escanea el topic /sensor_data ~5s para detectar MACs activas
 3. Si una MAC activa NO está en la BD → la registra automáticamente
 4. Menú de selección de dispositivo a calibrar
 5. Espacio para iniciar medición (sensor en buffer conocido)
 6. 10 muestras de estabilización (descartadas)
 7. 30 muestras reales → calcula mediana de voltaje (solo del ESP elegido)
 8. El usuario ingresa el pH real de esa solución buffer
 9. Repetir para cada buffer (mínimo 2, recomendado 3)
10. Al salir: regresión lineal con numpy → genera valores para config.h

Topic: /sensor_data (Float32MultiArray)
  [0] temperatura
  [1] pH calculado (con calibración anterior)
  [2] voltage_raw_ph  ← lo que usamos para la nueva regresión
  [3] mac_part1       ← (mac[0]<<16)|(mac[1]<<8)|mac[2]  como float
  [4] mac_part2       ← (mac[3]<<16)|(mac[4]<<8)|mac[5]  como float
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import sys
import os
import termios
import tty
import statistics
import numpy as np
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Dict, List

import time

# ── MongoDB ────────────────────────────────────────────────────────────────────
from dotenv import load_dotenv
from pymongo import MongoClient
from pymongo.errors import ConnectionFailure, DuplicateKeyError

# Cargar .env de la carpeta database/ (relativo a este script)
_DB_ENV = Path(__file__).parent.parent.parent.parent / 'database' / '.env'
load_dotenv(_DB_ENV)

_MONGO_URI     = os.getenv('MONGO_URI')
_MONGO_DB      = os.getenv('MONGO_DB', 'Datos_ESP')
_COL_DISP      = os.getenv('MONGO_COLLECTION_DISPOSITIVOS', 'devices_data')

# ── Constantes de escaneo ──────────────────────────────────────────────────────
SCAN_SECONDS = 5          # Segundos escaneando el topic antes de mostrar menú


# ==============================================================================
# UTILIDADES MAC
# ==============================================================================

def floats_to_mac(part1_f: float, part2_f: float) -> str:
    """
    Reconstruye el ID de dispositivo 'AABBCCDDEEFF' (sin separadores)
    desde los dos floats que publica el firmware ESP32.

    Firmware C:
        part1 = (mac[0]<<16)|(mac[1]<<8)|mac[2]
        part2 = (mac[3]<<16)|(mac[4]<<8)|mac[5]

    Se usa sin separadores para coincidir con el formato ya existente en MongoDB.
    """
    p1 = int(part1_f) & 0xFFFFFF
    p2 = int(part2_f) & 0xFFFFFF
    b = [
        (p1 >> 16) & 0xFF,
        (p1 >>  8) & 0xFF,
         p1        & 0xFF,
        (p2 >> 16) & 0xFF,
        (p2 >>  8) & 0xFF,
         p2        & 0xFF,
    ]
    return ''.join(f'{x:02X}' for x in b)


# ==============================================================================
# BASE DE DATOS
# ==============================================================================

def conectar_db():
    """Conecta a MongoDB y retorna (client, col_dispositivos) o (None, None)."""
    if not _MONGO_URI:
        print("  ⚠️  MONGO_URI no encontrado en .env — se omite BD")
        return None, None
    try:
        client = MongoClient(_MONGO_URI, serverSelectionTimeoutMS=5000)
        client.admin.command('ping')
        col = client[_MONGO_DB][_COL_DISP]
        return client, col
    except ConnectionFailure as e:
        print(f"  ⚠️  No se pudo conectar a MongoDB: {e}")
        return None, None


def listar_dispositivos_bd(col) -> List[Dict]:
    """Retorna dispositivos ordenados por fecha de registro (más reciente primero)."""
    if col is None:
        return []
    return list(col.find({}).sort("conexion.primera", -1))


def registrar_dispositivo_bd(col, mac: str) -> bool:
    """
    Registra un dispositivo nuevo en la BD.
    Retorna True si fue creado, False si ya existía.
    """
    if col is None:
        return False
    ahora = datetime.now(timezone.utc)
    doc = {
        "_id": mac,
        "nombre": f"Dispositivo {mac[-6:]}",
        "auto_registrado": True,
        "ubicacion": None,
        "conexion": {
            "primera": ahora,
            "ultima":  ahora,
            "total_lecturas": 0
        }
    }
    try:
        col.insert_one(doc)
        return True
    except DuplicateKeyError:
        return False


# ==============================================================================
# NODO ROS — ESCANEO Y CALIBRACIÓN
# ==============================================================================

class CalibrationNode(Node):
    """
    Nodo para calibración de pH.
    - Fase SCAN: detecta todos los MACs activos en el topic durante N segundos.
    - Fase CALIBRACIÓN: filtra por target_mac y acumula muestras de voltaje.
    """

    def __init__(self):
        super().__init__('ph_calibration')

        # ── Estado de escaneo ──────────────────────────────────────────────────
        self.scanning = False
        self.detected_macs: Dict[str, float] = {}   # mac_str → último voltaje

        # ── Estado de calibración ──────────────────────────────────────────────
        self.target_mac:     Optional[str] = None   # None = sin filtro
        self.voltage_samples: List[float] = []
        self.collecting      = False
        self.warmup_count    = 0
        self.sample_count    = 0
        self.last_sample_time = 0.0
        self.last_voltage:   Optional[float] = None

        # Configuración
        self.WARMUP_SAMPLES      = 10
        self.REAL_SAMPLES        = 30
        self.MIN_SAMPLE_INTERVAL = 0.3   # segundos

        self.sensor_sub = self.create_subscription(
            Float32MultiArray,
            'sensor_data',
            self._callback,
            10)

    # ── Callback único ─────────────────────────────────────────────────────────

    def _callback(self, msg):
        if len(msg.data) < 5:
            return

        voltage  = msg.data[2]
        mac_str  = floats_to_mac(msg.data[3], msg.data[4])

        # Fase SCAN: registrar MAC detectada
        if self.scanning:
            self.detected_macs[mac_str] = voltage

        # Fase CALIBRACIÓN: solo procesar el MAC objetivo
        if not self.collecting:
            return
        if self.target_mac is not None and mac_str != self.target_mac:
            return

        self.last_voltage = voltage
        now = time.time()
        if (now - self.last_sample_time) >= self.MIN_SAMPLE_INTERVAL:
            self.last_sample_time = now
            self._process_sample(voltage)

    def _process_sample(self, voltage: float):
        if self.warmup_count < self.WARMUP_SAMPLES:
            self.warmup_count += 1
            print(f"\r  🔄 Estabilizando... {self.warmup_count}/{self.WARMUP_SAMPLES}"
                  f"  [V: {voltage:.1f} mV]", end="", flush=True)
        elif self.sample_count < self.REAL_SAMPLES:
            self.voltage_samples.append(voltage)
            self.sample_count += 1
            print(f"\r  📊 Muestreando... {self.sample_count}/{self.REAL_SAMPLES}"
                  f" | V: {voltage:.1f} mV", end="", flush=True)

    # ── Control ────────────────────────────────────────────────────────────────

    def start_scan(self):
        self.detected_macs = {}
        self.scanning = True

    def stop_scan(self):
        self.scanning = False

    def start_collection(self):
        self.voltage_samples  = []
        self.warmup_count     = 0
        self.sample_count     = 0
        self.last_sample_time = 0.0
        self.collecting       = True

    def stop_collection(self):
        self.collecting = False

    def is_warmup_complete(self) -> bool:
        return self.warmup_count >= self.WARMUP_SAMPLES

    def is_complete(self) -> bool:
        return self.sample_count >= self.REAL_SAMPLES

    def get_median_voltage(self) -> Optional[float]:
        return statistics.median(self.voltage_samples) if self.voltage_samples else None


# ==============================================================================
# MENÚ DE DISPOSITIVOS
# ==============================================================================

def _fmt_fecha(doc: Dict) -> str:
    """Formatea la fecha de registro de un documento de dispositivo."""
    try:
        dt = doc.get("conexion", {}).get("primera")
        if dt:
            return dt.strftime('%Y-%m-%d') if hasattr(dt, 'strftime') else str(dt)[:10]
    except Exception:
        pass
    return "—"


def seleccionar_dispositivo(node: CalibrationNode, col) -> Optional[str]:
    """
    1. Consulta dispositivos en BD (ordenados por fecha desc).
    2. Escanea el topic SCAN_SECONDS segundos para hallar MACs activas.
    3. Registra en BD los MACs detectados que no estén registrados.
    4. Muestra menú y retorna el MAC elegido (o None = sin filtro).
    """

    # ── 1. Dispositivos en BD ──────────────────────────────────────────────────
    bd_devs = listar_dispositivos_bd(col)
    bd_ids  = {d['_id'] for d in bd_devs}

    # ── 2. Escaneo del topic ───────────────────────────────────────────────────
    print(f"\n  🔍 Escaneando topic /sensor_data por {SCAN_SECONDS}s...", end="", flush=True)
    node.start_scan()
    deadline = time.time() + SCAN_SECONDS
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        remaining = deadline - time.time()
        print(f"\r  🔍 Escaneando topic /sensor_data... {remaining:.0f}s restantes"
              f"  ({len(node.detected_macs)} detectado(s))", end="", flush=True)
    node.stop_scan()
    print()  # nueva línea

    # ── 3. Registrar MACs nuevos en BD ────────────────────────────────────────
    nuevos = []
    for mac in node.detected_macs:
        if mac not in bd_ids:
            registrado = registrar_dispositivo_bd(col, mac)
            if registrado:
                nuevos.append(mac)
                bd_ids.add(mac)
                print(f"  ✅ Nuevo dispositivo registrado en BD: {mac}")

    # Si hubo nuevos, recargar lista desde BD para tener doc completo
    if nuevos:
        bd_devs = listar_dispositivos_bd(col)

    # ── 4. Menú ────────────────────────────────────────────────────────────────
    activos = node.detected_macs  # mac → último voltaje visto

    print("\n" + "═"*70)
    print("  📡 SELECCIONAR DISPOSITIVO A CALIBRAR")
    print("═"*70)

    if not bd_devs and not activos:
        print("\n  ⚠️  No hay dispositivos en la BD ni activos en el topic.")
        print("       Asegurate de que el ESP32 esté publicando antes de continuar.\n")
        # Fallback sin filtro
        return _preguntar_sin_filtro()

    # Construir tabla unificada (BD + activos huérfanos)
    filas = []

    # Primero los que están en BD (ordenados por fecha)
    for doc in bd_devs:
        mac    = doc['_id']
        nombre = doc.get('nombre', '—')
        fecha  = _fmt_fecha(doc)
        activo = "🟢" if mac in activos else "⚫"
        filas.append((mac, nombre, fecha, activo))

    # MACs activos que no lograron registrarse en BD (si col es None)
    for mac in activos:
        if mac not in bd_ids:
            filas.append((mac, f"Dispositivo {mac[-8:]}", "—", "🟢"))

    print(f"\n  {'#':>3}  {'Estado':^6}  {'ID (MAC)':<20}  {'Nombre':<25}  {'Registrado':<12}")
    print("  " + "─"*72)
    for i, (mac, nombre, fecha, activo) in enumerate(filas, 1):
        print(f"  {i:>3}  {activo:^6}  {mac:<20}  {nombre:<25}  {fecha:<12}")

    print("  " + "─"*72)
    print(f"\n  {'0':>3}  — Sin filtro (acepta cualquier dispositivo — NO recomendado)")
    print()

    while True:
        try:
            raw = input("  Selecciona el número del dispositivo: ").strip()
            n = int(raw)
            if n == 0:
                print("  ⚠️  Modo sin filtro activo — se aceptarán datos de cualquier ESP.")
                return None
            if 1 <= n <= len(filas):
                mac_sel = filas[n - 1][0]
                nombre_sel = filas[n - 1][1]
                print(f"\n  ✅ Dispositivo seleccionado: {mac_sel}  ({nombre_sel})")
                if mac_sel not in activos:
                    print("  ⚠️  Este dispositivo NO está publicando actualmente.")
                    print("      Las mediciones esperarán hasta que lleguen datos de ese MAC.\n")
                return mac_sel
        except KeyboardInterrupt:
            print("\n\n  🛑 Cancelado por el usuario.")
            sys.exit(0)
        except ValueError:
            pass
        print("  Opción inválida, intenta de nuevo.")


def _preguntar_sin_filtro() -> Optional[str]:
    r = input("  ¿Continuar sin filtro de dispositivo? [s/N]: ").strip().lower()
    return None if r == 's' else sys.exit(0)


# ==============================================================================
# INTERACCIÓN CON TECLADO
# ==============================================================================

def get_key() -> str:
    """Lee una tecla del teclado sin esperar Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        return 'escape' if ch == '\x1b' else ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def ask_ph_value(median_v: float) -> Optional[float]:
    """Pide el pH real de la solución buffer (restaura terminal normal)."""
    print(f"\n\n  El voltaje medido fue: {median_v:.2f} mV")
    print("  Ingresa el pH REAL de esta solución buffer")
    print("  (ej: 4.01, 6.86, 9.18) y presiona Enter: ", end="", flush=True)
    try:
        return float(input().strip())
    except (ValueError, EOFError):
        return None


# ==============================================================================
# REGRESIÓN LINEAL
# ==============================================================================

def compute_regression(calibration_points):
    """
    Regresión lineal: pH = slope * voltage_mV + intercept
    calibration_points: lista de (voltage_mV, ph_real)
    """
    voltages  = np.array([p[0] for p in calibration_points])
    ph_values = np.array([p[1] for p in calibration_points])

    coeffs    = np.polyfit(voltages, ph_values, 1)
    slope, intercept = coeffs[0], coeffs[1]

    ph_pred   = np.polyval(coeffs, voltages)
    ss_res    = np.sum((ph_values - ph_pred) ** 2)
    ss_tot    = np.sum((ph_values - np.mean(ph_values)) ** 2)
    r_squared = 1 - (ss_res / ss_tot) if ss_tot != 0 else 1.0

    return slope, intercept, r_squared


def print_regression_results(calibration_points, slope, intercept, r_squared, target_mac):
    date_str    = datetime.now().strftime('%Y-%m-%d')
    buffers_str = ", ".join([f"pH {ph:.2f}→{v:.0f}mV" for v, ph in calibration_points])

    print("\n\n" + "="*70)
    print("  📐 REGRESIÓN LINEAL - RESULTADOS")
    print("="*70)

    if target_mac:
        print(f"\n  Dispositivo calibrado : {target_mac}")
    print(f"  Puntos de calibración : {len(calibration_points)}")
    print(f"\n  {'Voltaje (mV)':>15}  {'pH buffer':>10}  {'pH predicho':>12}  {'Error':>8}")
    print("  " + "─"*56)

    for v, ph_real in calibration_points:
        ph_pred = slope * v + intercept
        error   = ph_real - ph_pred
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

    print("\n\n  ┌─────────────────────────────────────────────────────────────┐")
    print("  │  📋 COPIAR EN config.h                                       │")
    print("  ├─────────────────────────────────────────────────────────────┤")
    print(f"  │  // pH: calibrado {date_str:<43}│")
    print(f"  │  // Buffers: {buffers_str:<49}│")
    print(f"  │  #define PH_SLOPE       {slope:<38.6f}│")
    print(f"  │  #define PH_INTERCEPT   {intercept:<38.6f}│")
    print("  └─────────────────────────────────────────────────────────────┘")


# ==============================================================================
# MAIN
# ==============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()

    print("\n" + "="*70)
    print("  🧪 HERRAMIENTA DE CALIBRACIÓN pH")
    print("  Topic: /sensor_data[2] = voltage_raw_ph")
    print("="*70)

    # ── Conectar BD ────────────────────────────────────────────────────────────
    db_client, col_disp = conectar_db()
    if col_disp is not None:
        print(f"  ✅ Conectado a MongoDB: {_MONGO_DB}.{_COL_DISP}")
    else:
        print("  ⚠️  Sin conexión a BD — no se mostrará lista de dispositivos")

    # ── Selección de dispositivo ───────────────────────────────────────────────
    target_mac = seleccionar_dispositivo(node, col_disp)
    node.target_mac = target_mac

    # ── Instrucciones ──────────────────────────────────────────────────────────
    print("\n" + "─"*70)
    print("\n  📋 Instrucciones:")
    print("     1. Sumerge el sensor en la solución buffer")
    print("     2. Presiona ESPACIO para iniciar captura")
    print("     3. Ingresa el pH real de esa solución cuando se pida")
    print("     4. Repite para cada buffer (mínimo 2, ideal 3)")
    print("     5. Presiona ENTER o ESCAPE para calcular la regresión")
    if target_mac:
        print(f"\n  🔒 Filtrando solo: {target_mac}")
    else:
        print("\n  ⚠️  Sin filtro — se aceptan datos de cualquier ESP")
    print("\n" + "─"*70)

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

                ph_real = ask_ph_value(median_v)
                if ph_real is None:
                    print("  ⚠️  pH inválido, medición descartada")
                    continue

                if not (0.0 <= ph_real <= 14.0):
                    print(f"  ⚠️  pH {ph_real} fuera de rango (0-14), medición descartada")
                    continue

                calibration_points.append((median_v, ph_real))
                print(f"\n  ✅ Punto guardado: {median_v:.2f} mV → pH {ph_real:.2f}")

                if len(calibration_points) > 1:
                    print("\n  📋 Puntos de calibración acumulados:")
                    print("     #   Voltaje (mV)    pH real")
                    print("    ─── ────────────  ──────────")
                    for i, (v, ph) in enumerate(calibration_points, 1):
                        print(f"    {i:2d}   {v:10.2f}       {ph:.2f}")

    except KeyboardInterrupt:
        pass

    # ── Regresión final ────────────────────────────────────────────────────────
    print("\n\n" + "="*70)
    print("  📊 RESUMEN FINAL DE CALIBRACIÓN")
    print("="*70)

    if len(calibration_points) < 2:
        print(f"\n  ⚠️  Se necesitan al menos 2 puntos para la regresión.")
        print(f"     Puntos capturados: {len(calibration_points)}")
    else:
        slope, intercept, r_squared = compute_regression(calibration_points)
        print_regression_results(calibration_points, slope, intercept, r_squared, target_mac)

    print("\n" + "="*70)
    print("  🛑 Programa terminado")
    print("="*70 + "\n")

    # ── Limpieza ───────────────────────────────────────────────────────────────
    if db_client:
        db_client.close()

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
