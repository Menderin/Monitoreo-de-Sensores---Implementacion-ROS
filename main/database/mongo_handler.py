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
                'timestamp': {'$gte': tiempo_limite.isoformat()}
            }
            
            datos = list(collection.find(query).sort('timestamp', 1))
            
            if not datos:
                return pd.DataFrame()
            
            # Procesar datos
            registros = []
            for registro in datos:
                try:
                    registros.append({
                        'timestamp': pd.to_datetime(registro.get('timestamp')),
                        'ph': registro.get('ph', {}).get('valor'),
                        'temperatura': registro.get('temperatura', {}).get('valor')
                    })
                except:
                    continue
            
            df = pd.DataFrame(registros)
            df = df.sort_values('timestamp').reset_index(drop=True)
            
            return df
        except Exception as e:
            st.error(f"Error al cargar datos: {e}")
            return pd.DataFrame()
    
    @staticmethod
    def limpiar_cache():
        """Limpia el cache de datos"""
        st.cache_data.clear()
