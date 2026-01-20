"""
Servicio de base de datos para sensores IoT.
Proporciona funciones para guardar lecturas y gestionar dispositivos.
"""

from datetime import datetime
from typing import Optional, Dict, Any, List
from pymongo import MongoClient
from pymongo.errors import ConnectionFailure, DuplicateKeyError

from .config import MongoConfig


class SensorDBService:
    """
    Servicio para interactuar con MongoDB.
    Maneja lecturas de sensores y registro de dispositivos.
    """
    
    _instance = None
    _client = None
    
    def __new__(cls):
        """Singleton para reutilizar conexión."""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        if self._client is None:
            self._conectar()
    
    def _conectar(self):
        """Establece conexión con MongoDB."""
        MongoConfig.validate()
        self._client = MongoClient(
            MongoConfig.URI,
            serverSelectionTimeoutMS=MongoConfig.TIMEOUT_MS
        )
        self._db = self._client[MongoConfig.DB_NAME]
        self._col_datos = self._db[MongoConfig.COL_DATOS]
        self._col_dispositivos = self._db[MongoConfig.COL_DISPOSITIVOS]
    
    def ping(self) -> bool:
        """Verifica conexión a MongoDB."""
        try:
            self._client.admin.command('ping')
            return True
        except ConnectionFailure:
            return False
    
    # ═══════════════════════════════════════════════════════════════
    # FUNCIONES PARA DATOS DE SENSORES
    # ═══════════════════════════════════════════════════════════════
    
    def guardar_lectura(
        self,
        dispositivo_id: str,
        temperatura: Optional[float] = None,
        ph: Optional[float] = None,
        timestamp: Optional[datetime] = None,
        telemetria: Optional[Dict] = None
    ) -> str:
        """
        Guarda una lectura de sensores.
        
        Args:
            dispositivo_id: ID del dispositivo (MAC o identificador)
            temperatura: Valor de temperatura
            ph: Valor de pH
            timestamp: Momento de la lectura (default: ahora)
            telemetria: Datos adicionales (rssi, heap, etc)
            
        Returns:
            ID del documento insertado
        """
        # Obtener calibración del dispositivo
        calibracion = self._obtener_calibracion(dispositivo_id)
        
        # Aplicar calibración
        temp_calibrada = None
        ph_calibrado = None
        
        if temperatura is not None:
            temp_calibrada = temperatura + calibracion.get('temp_offset', 0)
        
        if ph is not None:
            ph_calibrado = ph + calibracion.get('ph_offset', 0)
        
        # Crear documento
        documento = {
            "timestamp": timestamp or datetime.utcnow(),
            "dispositivo_id": dispositivo_id,
            "datos": {}
        }
        
        if temp_calibrada is not None:
            documento["datos"]["temperatura"] = round(temp_calibrada, 2)
        
        if ph_calibrado is not None:
            documento["datos"]["ph"] = round(ph_calibrado, 2)
        
        if telemetria:
            documento["telemetria"] = telemetria
        
        # Insertar
        resultado = self._col_datos.insert_one(documento)
        
        # Actualizar estadísticas del dispositivo
        self._actualizar_estadisticas_dispositivo(dispositivo_id)
        
        return str(resultado.inserted_id)
    
    def obtener_ultimas_lecturas(
        self,
        dispositivo_id: Optional[str] = None,
        limite: int = 100
    ) -> List[Dict]:
        """
        Obtiene las últimas lecturas.
        
        Args:
            dispositivo_id: Filtrar por dispositivo (opcional)
            limite: Número máximo de resultados
            
        Returns:
            Lista de documentos
        """
        filtro = {}
        if dispositivo_id:
            filtro["dispositivo_id"] = dispositivo_id
        
        cursor = self._col_datos.find(filtro).sort("timestamp", -1).limit(limite)
        return list(cursor)
    
    # ═══════════════════════════════════════════════════════════════
    # FUNCIONES PARA DISPOSITIVOS
    # ═══════════════════════════════════════════════════════════════
    
    def registrar_dispositivo(
        self,
        dispositivo_id: str,
        nombre: Optional[str] = None,
        auto_registrado: bool = True
    ) -> bool:
        """
        Registra un nuevo dispositivo o actualiza si ya existe.
        
        Args:
            dispositivo_id: ID del dispositivo (MAC)
            nombre: Nombre descriptivo
            auto_registrado: Si fue detectado automáticamente
            
        Returns:
            True si fue creado, False si ya existía
        """
        ahora = datetime.utcnow()
        
        dispositivo = {
            "_id": dispositivo_id,
            "nombre": nombre or f"Dispositivo {dispositivo_id[-8:]}",
            "estado": "pendiente" if auto_registrado else "activo",
            "auto_registrado": auto_registrado,
            "ubicacion": None,
            "configuracion": {
                "firmware_version": "desconocido",
                "intervalo_lectura_seg": 60,
                "sensores_habilitados": ["ph", "temperatura"]
            },
            "calibracion": {
                "ph_offset": 0.0,
                "temp_offset": 0.0
            },
            "unidades": {
                "temperatura": "°C",
                "ph": "pH"
            },
            "conexion": {
                "primera": ahora,
                "ultima": ahora,
                "total_lecturas": 0
            }
        }
        
        try:
            self._col_dispositivos.insert_one(dispositivo)
            print(f"Dispositivo registrado: {dispositivo_id}")
            return True
        except DuplicateKeyError:
            # Ya existe, solo actualizar última conexión
            self._col_dispositivos.update_one(
                {"_id": dispositivo_id},
                {"$set": {"conexion.ultima": ahora}}
            )
            return False
    
    def obtener_dispositivo(self, dispositivo_id: str) -> Optional[Dict]:
        """Obtiene información de un dispositivo."""
        return self._col_dispositivos.find_one({"_id": dispositivo_id})
    
    def listar_dispositivos(self, estado: Optional[str] = None) -> List[Dict]:
        """Lista todos los dispositivos."""
        filtro = {}
        if estado:
            filtro["estado"] = estado
        return list(self._col_dispositivos.find(filtro))
    
    def actualizar_calibracion(
        self,
        dispositivo_id: str,
        ph_offset: Optional[float] = None,
        temp_offset: Optional[float] = None
    ) -> bool:
        """
        Actualiza la calibración de un dispositivo.
        
        Args:
            dispositivo_id: ID del dispositivo
            ph_offset: Nuevo offset de pH
            temp_offset: Nuevo offset de temperatura
            
        Returns:
            True si se actualizó correctamente
        """
        actualizaciones = {}
        
        if ph_offset is not None:
            actualizaciones["calibracion.ph_offset"] = ph_offset
        
        if temp_offset is not None:
            actualizaciones["calibracion.temp_offset"] = temp_offset
        
        if not actualizaciones:
            return False
        
        resultado = self._col_dispositivos.update_one(
            {"_id": dispositivo_id},
            {"$set": actualizaciones}
        )
        
        return resultado.modified_count > 0
    
    def cambiar_estado_dispositivo(self, dispositivo_id: str, estado: str) -> bool:
        """
        Cambia el estado de un dispositivo.
        Estados válidos: pendiente, activo, mantenimiento, inactivo
        """
        estados_validos = ["pendiente", "activo", "mantenimiento", "inactivo"]
        if estado not in estados_validos:
            raise ValueError(f"Estado inválido. Usar: {estados_validos}")
        
        resultado = self._col_dispositivos.update_one(
            {"_id": dispositivo_id},
            {"$set": {"estado": estado}}
        )
        return resultado.modified_count > 0
    
    # ═══════════════════════════════════════════════════════════════
    # FUNCIONES PRIVADAS
    # ═══════════════════════════════════════════════════════════════
    
    def _obtener_calibracion(self, dispositivo_id: str) -> Dict:
        """Obtiene valores de calibración del dispositivo."""
        dispositivo = self.obtener_dispositivo(dispositivo_id)
        if dispositivo and "calibracion" in dispositivo:
            return dispositivo["calibracion"]
        return {"ph_offset": 0.0, "temp_offset": 0.0}
    
    def _actualizar_estadisticas_dispositivo(self, dispositivo_id: str):
        """Actualiza contador de lecturas y última conexión."""
        self._col_dispositivos.update_one(
            {"_id": dispositivo_id},
            {
                "$set": {"conexion.ultima": datetime.utcnow()},
                "$inc": {"conexion.total_lecturas": 1}
            }
        )
    
    def cerrar(self):
        """Cierra la conexión."""
        if self._client:
            self._client.close()
            SensorDBService._client = None
            SensorDBService._instance = None
