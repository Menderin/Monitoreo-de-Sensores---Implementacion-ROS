"""Gestión de conexión y consultas a MongoDB"""
import streamlit as st
import pandas as pd
from pymongo import MongoClient
from datetime import datetime, timedelta
from config.settings import Settings

class MongoHandler:
    """Manejador de conexión y consultas a MongoDB"""
    
    @staticmethod
    @st.cache_resource
    def get_client():
        """Conecta a MongoDB y retorna el cliente (cacheado)"""
        try:
            Settings.validate()
            client = MongoClient(Settings.MONGO_URI)
            return client
        except Exception as e:
            st.error(f"Error al conectar con MongoDB: {e}")
            return None
    
    @staticmethod
    @st.cache_data(ttl=Settings.CACHE_TTL)
    def cargar_datos(horas=1):
        """Carga datos para el dashboard (últimas N horas)"""
        # ... (Tu código original se mantiene igual aquí para el dashboard) ...
        try:
            client = MongoHandler.get_client()
            if client is None: return pd.DataFrame()
            db = client[Settings.MONGO_DB]
            collection = db[Settings.MONGO_COLLECTION]
            tiempo_limite = datetime.now() - timedelta(hours=horas)
            query = {'timestamp': {'$gte': tiempo_limite}}
            datos = list(collection.find(query).sort('timestamp', 1))
            if not datos: return pd.DataFrame()
            registros = []
            for registro in datos:
                try:
                    temp = registro.get('datos', {}).get('temperatura')
                    ph = registro.get('datos', {}).get('ph')
                    if temp is not None and ph is not None:
                        timestamp_utc = pd.to_datetime(registro.get('timestamp'))
                        timestamp_chile = timestamp_utc.tz_localize('UTC').tz_convert('America/Santiago')
                        registros.append({
                            'timestamp': timestamp_chile,
                            'dispositivo_id': registro.get('dispositivo_id', 'unknown'),
                            'temperatura': temp,
                            'ph': ph
                        })
                except: continue
            df = pd.DataFrame(registros)
            if df.empty: return df
            return df.sort_values('timestamp').reset_index(drop=True)
        except Exception as e:
            st.error(f"Error al cargar datos: {e}")
            return pd.DataFrame()

# ==========================================
# NUEVA FUNCIÓN OPTIMIZADA (Para Registros)
# ==========================================
@st.cache_data(ttl=60, show_spinner=False)
def cargar_datos_inteligente(limite=None, horas_atras=None):
    """
    Carga datos aplicando filtros DIRECTAMENTE en MongoDB.
    Esto hace que las vistas parciales (ej. últimos 50) sean instantáneas.
    
    Args:
        limite (int): Cantidad máxima de registros.
        horas_atras (int): Filtro de tiempo en horas.
    """
    try:
        client = MongoHandler.get_client()
        if client is None: return pd.DataFrame()
        
        db = client[Settings.MONGO_DB]
        collection = db[Settings.MONGO_COLLECTION]
        
        # 1. Construir Query (Filtro de Tiempo)
        query = {}
        if horas_atras:
            fecha_corte = datetime.utcnow() - timedelta(hours=horas_atras)
            query['timestamp'] = {'$gte': fecha_corte}
            
        # 2. Configurar Cursor (Ordenamiento)
        # Sort -1 (Descendente) para obtener los más recientes primero
        cursor = collection.find(query).sort('timestamp', -1)
        
        # 3. Aplicar Límite (Filtro de Cantidad) en la BD
        if limite:
            cursor = cursor.limit(limite)
            
        datos = list(cursor)
        
        if not datos: return pd.DataFrame()
        
        registros = []
        for registro in datos:
            try:
                temp = registro.get('datos', {}).get('temperatura')
                ph = registro.get('datos', {}).get('ph')
                
                if temp is not None and ph is not None:
                    # Manejo de fecha
                    timestamp_utc = pd.to_datetime(registro.get('timestamp'))
                    if timestamp_utc.tzinfo is None:
                        timestamp_chile = timestamp_utc.tz_localize('UTC').tz_convert('America/Santiago')
                    else:
                        timestamp_chile = timestamp_utc.tz_convert('America/Santiago')
                    
                    registros.append({
                        'timestamp': timestamp_chile,
                        'dispositivo_id': registro.get('dispositivo_id', 'unknown'),
                        'temperatura': temp,
                        'ph': ph
                    })
            except: continue
        
        return pd.DataFrame(registros)

    except Exception as e:
        st.error(f"Error en carga inteligente: {e}")
        return pd.DataFrame()

# Mantenemos la función antigua por si acaso, pero ya no la usaremos en Registros
@st.cache_data(ttl=300, show_spinner=False)
def cargar_todos_datos():
    return cargar_datos_inteligente(limite=None, horas_atras=None)

@st.cache_data(ttl=5, show_spinner=False)
def obtener_dispositivos():
    """Recupera la lista de dispositivos registrados y su configuración"""
    try:
        client = MongoHandler.get_client()
        if client is None: return []
        
        db = client[Settings.MONGO_DB]
        collection = db['devices_data']
        
        # Retornamos todos los documentos
        return list(collection.find({}))
    except Exception as e:
        st.error(f"Error al leer dispositivos: {e}")
        return []

def actualizar_dispositivo(device_id, datos_update):
    """Actualiza nombre, ubicación y umbrales de un dispositivo"""
    try:
        client = MongoHandler.get_client()
        if client is None: return False
        
        db = client[Settings.MONGO_DB]
        collection = db['devices_data']
        
        result = collection.update_one(
            {'_id': device_id},
            {'$set': datos_update}
        )
        
        # Limpiamos caché para ver los cambios reflejados al instante
        obtener_dispositivos.clear()
        return True
    except Exception as e:
        st.error(f"Error al actualizar: {e}")
        return False