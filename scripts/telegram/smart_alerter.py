#!/usr/bin/env python3
import os
import sys
import json
import time
import requests
from pathlib import Path
from datetime import datetime, timedelta, timezone

# ==========================================
# 1. CONFIGURACIÓN DE ENTORNO
# ==========================================
STARTUP_DELAY = int(os.getenv("ALERTER_STARTUP_DELAY", "15"))
print(f"[INFO] Esperando {STARTUP_DELAY}s para estabilización de red...")
time.sleep(STARTUP_DELAY)

try:
    from pymongo import MongoClient
    from dotenv import load_dotenv
except ImportError as e:
    print(f"[FATAL] Dependencia Python faltante: {e}")
    sys.exit(1)

env_path = Path(__file__).resolve().parents[2] / 'database' / '.env'
load_dotenv(dotenv_path=env_path)

MONGO_URI = os.getenv("MONGO_URI")
MONGO_DB_NAME = os.getenv("MONGO_DB")
MONGO_COL_SENSORS = os.getenv("MONGO_COLLECTION")
MONGO_COL_DEVICES = os.getenv("MONGO_COLLECTION_DISPOSITIVOS")
TOKEN = os.getenv("TELEGRAM_BOT_TOKEN")
CHAT_ID = os.getenv("TELEGRAM_CHAT_ID")

# ==========================================
# 2. CONEXIÓN A MONGODB
# ==========================================
MAX_RETRIES = 5
client = MongoClient(MONGO_URI, serverSelectionTimeoutMS=10000)
db = client[MONGO_DB_NAME]
col_devices = db[MONGO_COL_DEVICES]
col_sensors = db[MONGO_COL_SENSORS]

ESTADO_ALERTAS_FILE = str(Path(__file__).resolve().parent / 'alerter_estado.json')

def _cargar_estado_alertas():
    if os.path.exists(ESTADO_ALERTAS_FILE):
        try:
            with open(ESTADO_ALERTAS_FILE, 'r') as f: return json.load(f)
        except: return {}
    return {}

def _guardar_estado_alertas(estado):
    with open(ESTADO_ALERTAS_FILE, 'w') as f: json.dump(estado, f)

estado_alertas = _cargar_estado_alertas()
out_of_bounds_counter = {}
last_processed_ts = {}

def enviar_telegram(mensaje):
    url = f"https://api.telegram.org/bot{TOKEN}/sendMessage"
    payload = {"chat_id": CHAT_ID, "text": mensaje, "parse_mode": "Markdown"}
    try:
        requests.post(url, json=payload, timeout=10).raise_for_status()
        return True
    except: return False

def adaptar_dispositivo(dev):
    umbrales_raw = dev.get('umbrales', {})
    unidades_raw = dev.get('unidades', {})
    umbrales_norm = {}
    for k, v in umbrales_raw.items():
        if isinstance(v, dict): umbrales_norm[k] = v.copy()
        else:
            if k.endswith('_max'): umbrales_norm.setdefault(k[:-4], {})['critical_max'] = v
            elif k.endswith('_min'): umbrales_norm.setdefault(k[:-4], {})['critical_min'] = v
    for sk, ud in umbrales_norm.items():
        esp = "temperatura" if sk == "temperature" else sk
        if 'unit' not in ud: ud['unit'] = unidades_raw.get(esp, '')
    dev['umbrales'] = umbrales_norm
    return dev

def check_alerts():
    global out_of_bounds_counter, last_processed_ts
    dispositivos = col_devices.find()
    for dev in dispositivos:
        dev = adaptar_dispositivo(dev)
        dev_id = dev['_id']
        alias = dev.get('nombre', dev.get('alias', dev_id))
        umbrales = dev.get('umbrales', {})
        sensores_hab = dev.get('configuracion', {}).get('sensores_habilitados', [])

        for sensor in sensores_hab:
            sensor_key = "temperature" if sensor == "temperatura" else sensor
            umbral = umbrales.get(sensor_key, {})
            max_val = umbral.get('critical_max')
            min_val = umbral.get('critical_min')
            unidad = umbral.get('unit', '')

            if max_val is None or min_val is None: continue

            clave_memoria = f"{dev_id}_{sensor}"
            if clave_memoria not in out_of_bounds_counter: out_of_bounds_counter[clave_memoria] = 0

            query = {"dispositivo_id": dev_id}
            if clave_memoria in last_processed_ts:
                query["timestamp"] = {"$gt": last_processed_ts[clave_memoria]}
            else:
                query["timestamp"] = {"$gte": datetime.now(timezone.utc).replace(tzinfo=None) - timedelta(minutes=2)}

            lecturas = list(col_sensors.find(query).sort("timestamp", 1))
            for doc in lecturas:
                val = doc.get('datos', {}).get(sensor)
                if val is None: continue
                last_processed_ts[clave_memoria] = doc['timestamp']
                est_ant = estado_alertas.get(clave_memoria, "normal")

                # --- LÓGICA DE ALERTA ---
                if val < min_val or val > max_val:
                    out_of_bounds_counter[clave_memoria] += 1
                    if out_of_bounds_counter[clave_memoria] >= 15 and est_ant == "normal":
                        estado_actual = "alta" if val > max_val else "baja"
                        msg = f"🚨 *ALERTA CRÍTICA* 🚨\n📡 Dispositivo: *{alias}*\n⚠️ Parámetro: *{sensor.capitalize()}* ({estado_actual})\n📉 Rango: {min_val}{unidad} - {max_val}{unidad}\n🌡️ Valor: *{val:.2f}{unidad}*"
                        if enviar_telegram(msg):
                            estado_alertas[clave_memoria] = estado_actual
                            _guardar_estado_alertas(estado_alertas)
                        out_of_bounds_counter[clave_memoria] = 0
                
                # --- LÓGICA DE NORMALIZACIÓN CON HISTÉRESIS ---
                else:
                    margen = 0.5 if sensor == "temperatura" else 0.1
                    se_normalizo = False
                    if est_ant == "baja" and val >= (min_val + margen): se_normalizo = True
                    elif est_ant == "alta" and val <= (max_val - margen): se_normalizo = True
                    
                    if se_normalizo:
                        msg = f"✅ *SITUACIÓN NORMALIZADA* ✅\n📡 Dispositivo: *{alias}*\n👍 *{sensor.capitalize()}* volvió a rango seguro.\n🌡️ Valor: *{val:.2f}{unidad}*"
                        if enviar_telegram(msg):
                            estado_alertas[clave_memoria] = "normal"
                            _guardar_estado_alertas(estado_alertas)
                    if val >= min_val and val <= max_val:
                        out_of_bounds_counter[clave_memoria] = 0

if __name__ == "__main__":
    print("Iniciando monitoreo de alertas con Histéresis...")
    while True:
        try: check_alerts()
        except Exception as e: print(f"[ERROR] {e}"); time.sleep(10)
        time.sleep(10)
