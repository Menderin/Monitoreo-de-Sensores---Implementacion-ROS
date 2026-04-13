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

# Sleep inicial: da tiempo a que la red esté completamente operativa
# Esto evita que el script crashee al inicio cuando systemd lo arranca
# antes de que NetworkManager tenga conectividad plena.
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

# Ruta dinámica al .env (scripts/telegram/ → repo_root/database/.env)
env_path = Path(__file__).resolve().parents[2] / 'database' / '.env'
if not env_path.exists():
    print(f"[FATAL] .env no encontrado: {env_path}")
    print("[FATAL] Crea database/.env con las credenciales necesarias.")
    sys.exit(1)

load_dotenv(dotenv_path=env_path)

MONGO_URI = os.getenv("MONGO_URI")
MONGO_DB_NAME = os.getenv("MONGO_DB")
MONGO_COL_SENSORS = os.getenv("MONGO_COLLECTION")
MONGO_COL_DEVICES = os.getenv("MONGO_COLLECTION_DISPOSITIVOS")
TOKEN = os.getenv("TELEGRAM_BOT_TOKEN")
CHAT_ID = os.getenv("TELEGRAM_CHAT_ID")

# Validar variables críticas antes de continuar
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
        # Forzar conexión real para verificar conectividad
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

# Memoria persistente para no spamear alertas repetidas (sobrevive reinicios)
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

def check_alerts():
    global out_of_bounds_counter, last_processed_ts
    if col_devices is None or col_sensors is None:
        print("[WARN] Colecciones MongoDB no configuradas, saltando ciclo.")
        return

    dispositivos = col_devices.find()

    for dev in dispositivos:
        dev_id = dev['_id']
        alias = dev.get('alias', dev_id)
        umbrales = dev.get('umbrales', {})
        sensores_hab = dev.get('configuracion', {}).get('sensores_habilitados', [])

        for sensor in sensores_hab:
            # Buscar umbrales en inglés, buscar datos en español
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

            # Obtener lecturas nuevas
            query = {"dispositivo_id": dev_id}
            if clave_memoria in last_processed_ts:
                query["timestamp"] = {"$gt": last_processed_ts[clave_memoria]}
            else:
                # Si nunca se procesó, empezar desde hace 2 mins para no procesar toda la historia
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
                        # Alertar solo si acaba de salir de normal o si necesitamos reenviar (cooldown cumplido pero sin reingreso)
                        # Aquí, si mantenemos estado persistente, no mandamos SPAM, sólo 1 vez hasta volver a la normalidad
                        if estado_anterior == "normal":
                            msg = f"🚨 *ALERTA CRÍTICA* 🚨\n"
                            msg += f"📡 Dispositivo: *{alias}*\n"
                            msg += f"⚠️ Parámetro: *{sensor.capitalize()}* ({estado_actual})\n"
                            msg += f"📉 Rango crítico: {min_val}{unidad} - {max_val}{unidad}\n"
                            msg += f"🌡️ Último valor: *{val:.2f}{unidad}*"
                            if enviar_telegram(msg):
                                estado_alertas[clave_memoria] = estado_actual
                                _guardar_estado_alertas(estado_alertas)
                        # Cooldown: reiniciamos contador aunque siga fuera de rango
                        out_of_bounds_counter[clave_memoria] = 0

                else:
                    # Se normalizó el valor o fue un pico esporádico
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
        time.sleep(10)  # Revisión cada 10 segundos para procesar rápido las nuevas lecturas