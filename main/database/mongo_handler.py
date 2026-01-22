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
        """
        Carga datos de MongoDB de las últimas N horas
        
        Args:
            horas: Número de horas hacia atrás para consultar
            
        Returns:
            DataFrame con timestamp, ph y temperatura
        """
        try:
            client = MongoHandler.get_client()
            if client is None:
                return pd.DataFrame()
            
            db = client[Settings.MONGO_DB]
            collection = db[Settings.MONGO_COLLECTION]
            
            # Calcular tiempo límite
            tiempo_limite = datetime.now() - timedelta(hours=horas)
            
            # Consulta a MongoDB
            query = {
                'timestamp': {'$gte': tiempo_limite}
            }
            
            datos = list(collection.find(query).sort('timestamp', 1))
            
            if not datos:
                return pd.DataFrame()
            
            # Procesar datos
            registros = []
            for registro in datos:
                try:
                    # Extraer valores del nuevo esquema
                    temp = registro.get('datos', {}).get('temperatura')
                    ph = registro.get('datos', {}).get('ph')
                    
                    # Solo agregar si ambos valores existen
                    if temp is not None and ph is not None:
                        # Convertir timestamp de UTC a hora de Chile (UTC-3)
                        timestamp_utc = pd.to_datetime(registro.get('timestamp'))
                        timestamp_chile = timestamp_utc.tz_localize('UTC').tz_convert('America/Santiago')
                        
                        registros.append({
                            'timestamp': timestamp_chile,
                            'dispositivo_id': registro.get('dispositivo_id', 'unknown'),
                            'temperatura': temp,
                            'ph': ph
                        })
                except:
                    continue
            
            df = pd.DataFrame(registros)
            
            if df.empty:
                return df
            
            df = df.sort_values('timestamp').reset_index(drop=True)
            
            return df
        except Exception as e:
            st.error(f"Error al cargar datos: {e}")
            return pd.DataFrame()

# ==========================================
# FUNCIÓN DE CARGA COMPLETA (Para Registros)
# ==========================================
@st.cache_data(ttl=300, show_spinner=False)
def cargar_todos_datos():
    """
    Carga TODOS los datos de MongoDB sin límite de tiempo.
    Diseñado para la página de Registros.
    """
    try:
        client = MongoHandler.get_client()
        if client is None:
            return pd.DataFrame()
        
        db = client[Settings.MONGO_DB]
        collection = db[Settings.MONGO_COLLECTION]
        
        # Consulta sin filtro de tiempo - obtiene TODO
        datos = list(collection.find({}).sort('timestamp', -1))
        
        if not datos:
            return pd.DataFrame()
        
        registros = []
        for registro in datos:
            try:
                temp = registro.get('datos', {}).get('temperatura')
                ph = registro.get('datos', {}).get('ph')
                
                if temp is not None and ph is not None:
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
            except:
                continue
        
        df = pd.DataFrame(registros)
        if df.empty: return df
        df = df.sort_values('timestamp', ascending=False).reset_index(drop=True)
        return df
    except Exception as e:
        st.error(f"Error al cargar todos los datos: {e}")
        return pd.DataFrame()
    
    @staticmethod
    def limpiar_cache():
        """Limpia el cache de datos"""
        st.cache_data.clear()
