"""
Módulo de base de datos para sensores IoT.
Centraliza toda la lógica de conexión y operaciones con MongoDB.

Uso:
    from database.modules import SensorDBService, MongoConfig
    
    db = SensorDBService()
    db.guardar_lectura("esp32_001", temperatura=25.5, ph=7.2)
"""

from .modules.config import MongoConfig
from .modules.service import SensorDBService

__all__ = ['MongoConfig', 'SensorDBService']
