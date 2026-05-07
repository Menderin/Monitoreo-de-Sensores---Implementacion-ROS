import os
import json
import sys
import time
import requests
import argparse
from pathlib import Path
from datetime import datetime, timedelta, timezone

try:
    from pymongo import MongoClient
    from dotenv import load_dotenv
    import pandas as pd
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
ESTADO_FILE = str(Path(__file__).resolve().parent / 'reporte_estado.json')

def enviar_telegram(mensaje):
    url = f"https://api.telegram.org/bot{TOKEN}/sendMessage"
    payload = {"chat_id": CHAT_ID, "text": mensaje, "parse_mode": "Markdown"}
    try:
        requests.post(url, json=payload, timeout=15).raise_for_status()
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

def generar_reporte():
    ahora = datetime.now()
    fin_local = ahora.replace(minute=0, second=0, microsecond=0)
    inicio_local = fin_local - timedelta(hours=12)
    tipo = "🌅 Reporte de Mañana" if ahora.hour < 14 else "🌃 Reporte de Tarde"
    
    client = MongoClient(MONGO_URI, serverSelectionTimeoutMS=10000)
    db = client[MONGO_DB_NAME]
    col_devices = db[MONGO_COL_DEVICES]
    col_sensors = db[MONGO_COL_SENSORS]

    mensaje = f"📊 *{tipo}*\n📅 Periodo: {inicio_local.strftime('%H:%M')} a {fin_local.strftime('%H:%M')} hrs\n\n"
    dispositivos = col_devices.find()
    hay_datos = False

    for dev in dispositivos:
        dev = adaptar_dispositivo(dev)
        dev_id = dev['_id']
        alias = dev.get('alias') or dev.get('nombre') or dev_id
        
        query = {"dispositivo_id": dev_id, "timestamp": {"$gte": inicio_local.astimezone(timezone.utc).replace(tzinfo=None), "$lte": fin_local.astimezone(timezone.utc).replace(tzinfo=None)}}
        lecturas = list(col_sensors.find(query))
        
        if not lecturas: continue
        hay_datos = True
        mensaje += f"🔹 *Dispositivo: {alias}*\n  └ 📈 Lecturas: {len(lecturas)}\n```text\nVar     Med   Pro   Min   Max\n" + "-"*29 + "\n"
        df = pd.DataFrame([doc['datos'] for doc in lecturas if 'datos' in doc])
        
        for sensor in dev.get('configuracion', {}).get('sensores_habilitados', []):
            if sensor in df.columns:
                v_nom = "T(°C)" if sensor == "temperatura" else "pH"
                mensaje += f"{v_nom:<5} {df[sensor].median():>5.2f} {df[sensor].mean():>5.2f} {df[sensor].min():>5.2f} {df[sensor].max():>5.2f}\n"
        mensaje += "```\n\n"

    if hay_datos: enviar_telegram(mensaje)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--daily", action="store_true")
    args = parser.parse_args()
    if args.daily: generar_reporte()
