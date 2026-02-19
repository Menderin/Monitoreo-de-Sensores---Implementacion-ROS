"""
Configuración centralizada para conexión a MongoDB.
Todos los scripts deben importar desde aquí.
"""

import os
from pathlib import Path
from dotenv import load_dotenv

# Cargar .env desde la carpeta padre (database/)
ENV_PATH = Path(__file__).parent.parent / '.env'
load_dotenv(ENV_PATH)


class MongoConfig:
    """Configuración de MongoDB Atlas."""
    
    # Conexión
    URI = os.getenv('MONGO_URI')
    DB_NAME = os.getenv('MONGO_DB', 'Datos_ESP')
    
    # Colecciones
    COL_DATOS = os.getenv('MONGO_COLLECTION', 'data_sensors')
    COL_DISPOSITIVOS = os.getenv('MONGO_COLLECTION_DISPOSITIVOS', 'data_devices')
    
    # Timeouts
    TIMEOUT_MS = 5000
    
    @classmethod
    def validate(cls):
        """Valida que las variables estén configuradas."""
        if not cls.URI:
            raise ValueError("MONGO_URI no configurado en .env")
        if not cls.DB_NAME:
            raise ValueError("MONGO_DB no configurado en .env")
        return True
    
    @classmethod
    def get_info(cls):
        """Retorna información de conexión (sin credenciales)."""
        return {
            "database": cls.DB_NAME,
            "col_datos": cls.COL_DATOS,
            "col_dispositivos": cls.COL_DISPOSITIVOS
        }
