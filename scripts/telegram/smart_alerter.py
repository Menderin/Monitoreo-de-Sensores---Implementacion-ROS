import os
import time
import requests
from pathlib import Path
from pymongo import MongoClient
from datetime import datetime, timedelta, timezone
from dotenv import load_dotenv

# ==========================================
# 1. CONFIGURACIÓN DE ENTORNO
# ==========================================
# Ruta dinámica al .env (scripts/telegram/ → repo_root/database/.env)
env_path = Path(__file__).resolve().parents[2] / 'database' / '.env'
if not env_path.exists():
    raise FileNotFoundError(f".env no encontrado: {env_path}")
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
client = MongoClient(MONGO_URI, serverSelectionTimeoutMS=5000)
db = client[MONGO_DB_NAME]
col_devices = db[MONGO_COL_DEVICES]
col_sensors = db[MONGO_COL_SENSORS]

# Memoria para no spamear alertas repetidas
estado_alertas = {}

def enviar_telegram(mensaje):
    url = f"https://api.telegram.org/bot{TOKEN}/sendMessage"
    payload = {"chat_id": CHAT_ID, "text": mensaje, "parse_mode": "Markdown"}
    try:
        respuesta = requests.post(url, json=payload, timeout=10)
        respuesta.raise_for_status()
    except requests.exceptions.RequestException as e:
        print(f"[WARN] No se pudo enviar mensaje de Telegram: {e}")

def check_alerts():
    ahora_utc = datetime.now(timezone.utc).replace(tzinfo=None)
    hace_2_minutos = ahora_utc - timedelta(minutes=2)

    dispositivos = col_devices.find()

    for dev in dispositivos:
        dev_id = dev['_id']
        alias = dev.get('alias', dev_id)
        umbrales = dev.get('umbrales', {})
        sensores_hab = dev.get('configuracion', {}).get('sensores_habilitados', [])

        query = {
            "dispositivo_id": dev_id,
            "timestamp": {"$gte": hace_2_minutos, "$lte": ahora_utc}
        }
        lecturas = list(col_sensors.find(query).sort("timestamp", -1))

        if not lecturas:
            continue

        for sensor in sensores_hab:
            # Buscar umbrales en inglés, buscar datos en español
            sensor_key = "temperature" if sensor == "temperatura" else sensor
            umbral = umbrales.get(sensor_key, {})

            max_val = umbral.get('max_value')
            min_val = umbral.get('min_value')
            unidad = umbral.get('unit', '')

            if max_val is None or min_val is None:
                continue

            valores = [doc['datos'].get(sensor) for doc in lecturas if sensor in doc.get('datos', {})]
            valores = [v for v in valores if v is not None]

            if len(valores) == 0:
                continue

            estado_actual = "normal"
            if all(v > max_val for v in valores):
                estado_actual = "alta"
            elif all(v < min_val for v in valores):
                estado_actual = "baja"

            clave_memoria = f"{dev_id}_{sensor}"
            estado_anterior = estado_alertas.get(clave_memoria, "normal")

            # Si hay una NUEVA alerta
            if estado_actual != "normal" and estado_anterior == "normal":
                val_prom = sum(valores) / len(valores)
                msg = f"🚨 *ALERTA CRÍTICA* 🚨\n"
                msg += f"📡 Dispositivo: *{alias}*\n"
                msg += f"⚠️ Parámetro: *{sensor.capitalize()}* ({estado_actual})\n"
                msg += f"📉 Rango seguro: {min_val}{unidad} - {max_val}{unidad}\n"
                msg += f"🌡️ Valor actual (2 min): *{val_prom:.2f}{unidad}*"
                enviar_telegram(msg)
                estado_alertas[clave_memoria] = estado_actual

            # Si la situación se NORMALIZÓ
            elif estado_actual == "normal" and estado_anterior != "normal":
                ultima_lectura = valores[0]
                msg = f"✅ *SITUACIÓN NORMALIZADA* ✅\n"
                msg += f"📡 Dispositivo: *{alias}*\n"
                msg += f"👍 El parámetro *{sensor.capitalize()}* volvió a rango seguro.\n"
                msg += f"🌡️ Valor actual: *{ultima_lectura:.2f}{unidad}*"
                enviar_telegram(msg)
                estado_alertas[clave_memoria] = "normal"

if __name__ == "__main__":
    enviar_telegram("🛡️ *Sistema de Alertas Iniciado* | IoT Sensor Gateway.")
    print("Iniciando monitoreo de alertas... (Ctrl+C para detener)")
    while True:
        try:
            check_alerts()
        except Exception as e:
            print(f"Error en el ciclo: {e}")
        time.sleep(60)  # Revisión cada 1 minuto