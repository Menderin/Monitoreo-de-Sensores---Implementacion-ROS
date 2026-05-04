#!/usr/bin/env python3
import os
import sys
import json
import time
import requests
from pathlib import Path
from datetime import datetime, timedelta, timezone

# ==========================================
# 1. CONFIGURACIÓN DE ENTORNO (con manejo robusto de errores)
# ==========================================

STARTUP_DELAY = int(os.getenv("ALERTER_STARTUP_DELAY", "15"))
print(f"[INFO] Esperando {STARTUP_DELAY}s para estabilización de red...")
time.sleep(STARTUP_DELAY)

try:
    from pymongo import MongoClient
    from dotenv import load_dotenv
except ImportError as e:
    print(f"[FATAL] Dependencia Python faltante: {e}")
    print("[FATAL] Instala con: pip3 install pymongo python-dotenv --break-system-packages")
    sys.exit(1)

env_path = Path(__file__).resolve().parents[2] / 'database' / '.env'
if not env_path.exists():
    print(f"[FATAL] .env no encontrado: {env_path}")
    sys.exit(1)

load_dotenv(dotenv_path=env_path)

MONGO_URI = os.getenv("MONGO_URI")
MONGO_DB_NAME = os.getenv("MONGO_DB")
MONGO_COL_SENSORS = os.getenv("MONGO_COLLECTION")
MONGO_COL_DEVICES = os.getenv("MONGO_COLLECTION_DISPOSITIVOS")
TOKEN = os.getenv("TELEGRAM_BOT_TOKEN")
CHAT_ID = os.getenv("TELEGRAM_CHAT_ID")

_missing = []
if not MONGO_URI:
    _missing.append("MONGO_URI")
if not MONGO_DB_NAME:
    _missing.append("MONGO_DB")
if not TOKEN:
    _missing.append("TELEGRAM_BOT_TOKEN")
if not CHAT_ID:
    _missing.append("TELEGRAM_CHAT_ID")
if _missing:
    print(f"[FATAL] Variables de entorno faltantes en {env_path}: {', '.join(_missing)}")
    sys.exit(1)

# ==========================================
# 2. CONEXIÓN A MONGODB (con reintentos)
# ==========================================
MAX_RETRIES = 5
client = None
db = None
col_devices = None
col_sensors = None

for attempt in range(1, MAX_RETRIES + 1):
    try:
        client = MongoClient(MONGO_URI, serverSelectionTimeoutMS=10000)
        client.admin.command('ping')
        db = client[MONGO_DB_NAME]
        col_devices = db[MONGO_COL_DEVICES] if MONGO_COL_DEVICES else None
        col_sensors = db[MONGO_COL_SENSORS] if MONGO_COL_SENSORS else None
        print(f"[OK] Conectado a MongoDB (intento {attempt}/{MAX_RETRIES})")
        break
    except Exception as e:
        print(f"[WARN] MongoDB no disponible (intento {attempt}/{MAX_RETRIES}): {e}")
        if attempt == MAX_RETRIES:
            print("[FATAL] No se pudo conectar a MongoDB tras todos los reintentos.")
            sys.exit(1)
        time.sleep(10)

ESTADO_ALERTAS_FILE = str(Path(__file__).resolve().parent / 'alerter_estado.json')

def _cargar_estado_alertas():
    if os.path.exists(ESTADO_ALERTAS_FILE):
        try:
            with open(ESTADO_ALERTAS_FILE, 'r') as f:
                return json.load(f)
        except (json.JSONDecodeError, IOError):
            return {}
    return {}

def _guardar_estado_alertas(estado):
    try:
        with open(ESTADO_ALERTAS_FILE, 'w') as f:
            json.dump(estado, f)
    except IOError as e:
        print(f"[WARN] No se pudo guardar estado de alertas: {e}")

estado_alertas = _cargar_estado_alertas()
out_of_bounds_counter = {}
last_processed_ts = {}

def enviar_telegram(mensaje):
    """Envía mensaje a Telegram. Retorna True si fue exitoso, False si falló."""
    url = f"https://api.telegram.org/bot{TOKEN}/sendMessage"
    payload = {"chat_id": CHAT_ID, "text": mensaje, "parse_mode": "Markdown"}
    try:
        respuesta = requests.post(url, json=payload, timeout=10)
        respuesta.raise_for_status()
        return True
    except requests.exceptions.RequestException as e:
        print(f"[WARN] No se pudo enviar mensaje de Telegram: {e}")
        return False

def adaptar_dispositivo(dev):
    """
    Adapter Pattern: Estandariza la estructura del documento de MongoDB.
    Convierte arquitecturas planas (Tachi) a la arquitectura anidada original,
    garantizando retrocompatibilidad con otras implementaciones.
    """
    umbrales_raw = dev.get('umbrales', {})
    unidades_raw = dev.get('unidades', {})
    umbrales_norm = {}

    for k, v in umbrales_raw.items():
        if isinstance(v, dict):
            umbrales_norm[k] = v.copy()

    for k, v in umbrales_raw.items():
        if not isinstance(v, dict):
            if k.endswith('_max'):
                sensor = k[:-4]
                umbrales_norm.setdefault(sensor, {})['critical_max'] = v
            elif k.endswith('_min'):
                sensor = k[:-4]
                umbrales_norm.setdefault(sensor, {})['critical_min'] = v

    for sensor_k, umbral_dict in umbrales_norm.items():
        sensor_esp = "temperatura" if sensor_k == "temperature" else sensor_k
        if 'unit' not in umbral_dict and sensor_esp in unidades_raw:
            umbral_dict['unit'] = unidades_raw.get(sensor_esp, '')

    dev['umbrales'] = umbrales_norm
    return dev

def check_alerts():
    global out_of_bounds_counter, last_processed_ts
    if col_devices is None or col_sensors is None:
        print("[WARN] Colecciones MongoDB no configuradas, saltando ciclo.")
        return

    dispositivos = col_devices.find()

    for dev in dispositivos:
        dev = adaptar_dispositivo(dev)

        dev_id = dev['_id']
        alias = dev.get('alias', dev_id)
        umbrales = dev.get('umbrales', {})
        sensores_hab = dev.get('configuracion', {}).get('sensores_habilitados', [])

        for sensor in sensores_hab:
            sensor_key = "temperature" if sensor == "temperatura" else sensor
            umbral = umbrales.get(sensor_key, {})

            max_val = umbral.get('critical_max')
            min_val = umbral.get('critical_min')
            unidad = umbral.get('unit', '')

            if max_val is None or min_val is None:
                continue

            clave_memoria = f"{dev_id}_{sensor}"
            if clave_memoria not in out_of_bounds_counter:
                out_of_bounds_counter[clave_memoria] = 0

            query = {"dispositivo_id": dev_id}
            if clave_memoria in last_processed_ts:
                query["timestamp"] = {"$gt": last_processed_ts[clave_memoria]}
            else:
                ahora_utc = datetime.now(timezone.utc).replace(tzinfo=None)
                query["timestamp"] = {"$gte": ahora_utc - timedelta(minutes=2)}

            lecturas = list(col_sensors.find(query).sort("timestamp", 1))
            if not lecturas:
                continue

            for doc in lecturas:
                val = doc.get('datos', {}).get(sensor)
                if val is None:
                    continue

                last_processed_ts[clave_memoria] = doc['timestamp']
                estado_anterior = estado_alertas.get(clave_memoria, "normal")

                if val < min_val or val > max_val:
                    out_of_bounds_counter[clave_memoria] += 1

                    if out_of_bounds_counter[clave_memoria] >= 15:
                        estado_actual = "alta" if val > max_val else "baja"
                        if estado_anterior == "normal":
                            msg = f"🚨 *ALERTA CRÍTICA* 🚨\n"
                            msg += f"📡 Dispositivo: *{alias}*\n"
                            msg += f"⚠️ Parámetro: *{sensor.capitalize()}* ({estado_actual})\n"
                            msg += f"📉 Rango crítico: {min_val}{unidad} - {max_val}{unidad}\n"
                            msg += f"🌡️ Último valor: *{val:.2f}{unidad}*"
                            if enviar_telegram(msg):
                                estado_alertas[clave_memoria] = estado_actual
                                _guardar_estado_alertas(estado_alertas)
                        out_of_bounds_counter[clave_memoria] = 0

                else:
                    out_of_bounds_counter[clave_memoria] = 0
                    if estado_anterior != "normal":
                        msg = f"✅ *SITUACIÓN NORMALIZADA* ✅\n"
                        msg += f"📡 Dispositivo: *{alias}*\n"
                        msg += f"👍 El parámetro *{sensor.capitalize()}* volvió a rango seguro.\n"
                        msg += f"🌡️ Valor actual: *{val:.2f}{unidad}*"
                        if enviar_telegram(msg):
                            estado_alertas[clave_memoria] = "normal"
                            _guardar_estado_alertas(estado_alertas)

if __name__ == "__main__":
    try:
        enviar_telegram("🛡️ *Sistema de Alertas Iniciado* | IoT Sensor Gateway.")
    except Exception as e:
        print(f"[WARN] No se pudo enviar mensaje de inicio: {e}")

    print("Iniciando monitoreo de alertas... (Ctrl+C para detener)")
    while True:
        try:
            check_alerts()
        except Exception as e:
            print(f"[ERROR] Error en el ciclo de alertas: {e}")
            time.sleep(10)
        time.sleep(10)
