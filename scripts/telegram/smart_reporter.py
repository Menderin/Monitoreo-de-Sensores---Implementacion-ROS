import os
import json
import sys
import time
import requests
from pathlib import Path
from datetime import datetime, timedelta, timezone

try:
    from pymongo import MongoClient
    from dotenv import load_dotenv
    import pandas as pd
except ImportError as e:
    print(f"[FATAL] Dependencia Python faltante: {e}")
    print("[FATAL] Instala con: pip3 install pymongo python-dotenv pandas --break-system-packages")
    sys.exit(1)

# ==========================================
# 1. CONFIGURACIÓN DE ENTORNO
# ==========================================
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
ESTADO_FILE = str(Path(__file__).resolve().parent / 'reporte_estado.json')

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
# 2. FUNCIONES DE MEMORIA (Antispam)
# ==========================================
def cargar_estado():
    if os.path.exists(ESTADO_FILE):
        with open(ESTADO_FILE, 'r') as f:
            return json.load(f)
    return {}

def guardar_estado(estado):
    with open(ESTADO_FILE, 'w') as f:
        json.dump(estado, f)

def enviar_telegram(mensaje):
    """Envía mensaje a Telegram. Retorna True si fue exitoso, False si falló."""
    url = f"https://api.telegram.org/bot{TOKEN}/sendMessage"
    payload = {"chat_id": CHAT_ID, "text": mensaje, "parse_mode": "Markdown"}
    try:
        respuesta = requests.post(url, json=payload, timeout=15)
        respuesta.raise_for_status()
        return True
    except requests.exceptions.RequestException as e:
        print(f"[WARN] No se pudo enviar mensaje de Telegram: {e}")
        return False

# ==========================================
# 3. LÓGICA DE TURNOS (08:00 y 20:00)
# ==========================================
def obtener_ventana_tiempo():
    ahora = datetime.now()
    
    if 8 <= ahora.hour < 20:
        id_reporte = f"{ahora.strftime('%Y-%m-%d')}_manana"
        fin_local = ahora.replace(hour=8, minute=0, second=0, microsecond=0)
        tipo = "🌅 Reporte de Mañana" if ahora.hour == 8 else "🌅 Reporte de Mañana (Recuperado)"
            
    elif ahora.hour >= 20:
        id_reporte = f"{ahora.strftime('%Y-%m-%d')}_tarde"
        fin_local = ahora.replace(hour=20, minute=0, second=0, microsecond=0)
        tipo = "🌃 Reporte de Tarde" if ahora.hour == 20 else "🌃 Reporte de Tarde (Recuperado)"
            
    else:
        ayer = ahora - timedelta(days=1)
        id_reporte = f"{ayer.strftime('%Y-%m-%d')}_tarde"
        fin_local = ayer.replace(hour=20, minute=0, second=0, microsecond=0)
        tipo = "🌃 Reporte de Tarde (Recuperado)"

    inicio_local = fin_local - timedelta(hours=12)
    fin_utc = fin_local.astimezone(timezone.utc).replace(tzinfo=None)
    inicio_utc = inicio_local.astimezone(timezone.utc).replace(tzinfo=None)
    
    return id_reporte, inicio_utc, fin_utc, tipo, inicio_local, fin_local

# ==========================================
# 4. GENERACIÓN DEL REPORTE FINAL
# ==========================================
def generar_reporte():
    id_reporte, inicio_utc, fin_utc, tipo_reporte, inicio_local, fin_local = obtener_ventana_tiempo()
    estado = cargar_estado()
    
    if estado.get(id_reporte) == True:
        return

    # Conexión con reintentos (alineado con smart_alerter.py)
    MAX_RETRIES = 3
    client = None
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            client = MongoClient(MONGO_URI, serverSelectionTimeoutMS=10000)
            client.admin.command('ping')
            print(f"[OK] Conectado a MongoDB (intento {attempt}/{MAX_RETRIES})")
            break
        except Exception as e:
            print(f"[WARN] MongoDB no disponible (intento {attempt}/{MAX_RETRIES}): {e}")
            if attempt == MAX_RETRIES:
                print("[ERROR] No se pudo conectar a MongoDB. Reporte no generado.")
                return
            time.sleep(5)

    db = client[MONGO_DB_NAME]
    col_devices = db[MONGO_COL_DEVICES]
    col_sensors = db[MONGO_COL_SENSORS]

    mensaje = f"📊 *{tipo_reporte}*\n"
    mensaje += f"📅 Periodo: {inicio_local.strftime('%H:%M')} a {fin_local.strftime('%H:%M')} hrs\n\n"
    
    dispositivos = col_devices.find()
    datos_encontrados = False
    
    for dev in dispositivos:
        dev_id = dev['_id']
        alias = dev.get('alias', dev_id)
        sensores_hab = dev.get('configuracion', {}).get('sensores_habilitados', [])
        
        query = {
            "dispositivo_id": dev_id,
            "timestamp": {"$gte": inicio_utc, "$lte": fin_utc}
        }
        lecturas = list(col_sensors.find(query))
        
        if not lecturas:
            continue
            
        datos_encontrados = True
        mensaje += f"🔹 *Dispositivo: {alias}*\n"
        mensaje += f"  └ 📈 Total de lecturas: {len(lecturas)}\n"
        
        lista_datos = [doc['datos'] for doc in lecturas if 'datos' in doc]
        df = pd.DataFrame(lista_datos)
        
        # 🟢 INICIO DE LA TABLA COPIABLE (Modo Ultra-Compacto)
        mensaje += "```text\n"
        mensaje += "Var     Med   Pro   Min   Max\n"
        mensaje += "-"*29 + "\n"
        
        for sensor in sensores_hab:
            if sensor in df.columns and not df[sensor].isnull().all():
                s_min = df[sensor].min()
                s_max = df[sensor].max()
                s_prom = df[sensor].mean()
                s_med = df[sensor].median()
                
                # Abreviaturas extremas para no romper la pantalla
                if sensor == "temperatura":
                    var_nombre = "T(°C)"
                elif sensor == "ph":
                    var_nombre = "pH"
                else:
                    var_nombre = sensor[:4].capitalize()
                
                # Formato exprimido al milímetro (29 caracteres por línea)
                mensaje += f"{var_nombre:<5} {s_med:>5.2f} {s_prom:>5.2f} {s_min:>5.2f} {s_max:>5.2f}\n"
                
        mensaje += "```\n\n"
        # 🔴 FIN DE LA TABLA
        
    if not datos_encontrados:
        mensaje += "⚠️ No se registraron datos en este periodo.\n"
        
    if enviar_telegram(mensaje):
        estado[id_reporte] = True
        guardar_estado(estado)
    else:
        print(f"[WARN] Reporte {id_reporte} NO marcado como enviado (fallo Telegram). Se reintentara.")

if __name__ == "__main__":
    generar_reporte()