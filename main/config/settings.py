"""Configuración general de la aplicación"""
import os
from pathlib import Path
from dotenv import load_dotenv

# ─────────────────────────────────────────────────────────
# Cargar .env desde la carpeta database/ centralizada
# ─────────────────────────────────────────────────────────
db_env_path = Path(__file__).parent.parent.parent / 'database' / '.env'
load_dotenv(db_env_path)

class Settings:
    """Configuración centralizada de la aplicación"""
    
    # Configuración de la página
    PAGE_TITLE = "Sistema de Monitoreo Ambiental"
    PAGE_ICON = "🌊"
    LAYOUT = "wide"
    SIDEBAR_STATE = "expanded"
    
    # Configuración MongoDB
    MONGO_URI = os.getenv('MONGO_URI')
    MONGO_DB = os.getenv('MONGO_DB')
    MONGO_COLLECTION = os.getenv('MONGO_COLLECTION')
    
    # Configuración de cache
    CACHE_TTL = 10  # segundos
    
    # Configuración de auto-refresh
    AUTO_REFRESH_INTERVAL = 10  # segundos
    
    # Opciones de rango temporal
    RANGO_HORAS_OPTIONS = [1, 3, 6, 12, 24]
    
    # Opciones de registros para tabla
    REGISTROS_OPTIONS = [10, 25, 50, 100, "Todos"]
    
    @classmethod
    def validate(cls):
        """Valida que las variables de entorno estén configuradas"""
        if not cls.MONGO_URI:
            raise ValueError("MONGO_URI no está configurado en .env")
        if not cls.MONGO_DB:
            raise ValueError("MONGO_DB no está configurado en .env")
        if not cls.MONGO_COLLECTION:
            raise ValueError("MONGO_COLLECTION no está configurado en .env")
        return True
