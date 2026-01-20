#!/usr/bin/env python3
"""
Nodo ROS 2 principal para recibir datos de sensores y almacenarlos en MongoDB.

Este nodo:
- Suscribe a los tópicos /temperatura y /ph
- Aplica calibración automática desde la BD
- Guarda las lecturas en MongoDB (Time Series)
- Auto-registra dispositivos nuevos

Uso:
    python3 ros_sensor_node.py
"""

import sys
from pathlib import Path

# Agregar el directorio padre al path para importar database
sys.path.insert(0, str(Path(__file__).parent.parent))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from datetime import datetime

from database.modules import SensorDBService, MongoConfig


class SensorDBNode(Node):
    """
    Nodo ROS 2 que recibe datos de sensores y los almacena en MongoDB.
    """
    
    def __init__(self):
        super().__init__('sensor_db_node')
        
        # ═══════════════════════════════════════════════════════════
        # CONFIGURACIÓN
        # ═══════════════════════════════════════════════════════════
        self.DISPOSITIVO_ID = "esp32_default"  # ID del dispositivo
        self.INTERVALO_GUARDADO = 5.0  # Segundos entre guardados
        
        # Valores actuales de sensores
        self._temperatura_actual = None
        self._ph_actual = None
        self._ultimo_guardado = None
        
        # ═══════════════════════════════════════════════════════════
        # CONEXIÓN A BASE DE DATOS
        # ═══════════════════════════════════════════════════════════
        self.get_logger().info("Conectando a MongoDB...")
        try:
            self.db = SensorDBService()
            if self.db.ping():
                self.get_logger().info("[OK] Conectado a MongoDB Atlas")
                info = MongoConfig.get_info()
                self.get_logger().info(f"   Base de datos: {info['database']}")
                self.get_logger().info(f"   Colección datos: {info['col_datos']}")
            else:
                self.get_logger().error("[ERROR] No se pudo conectar a MongoDB")
                return
        except Exception as e:
            self.get_logger().error(f"[ERROR] Error de conexión: {e}")
            return
        
        # ═══════════════════════════════════════════════════════════
        # AUTO-REGISTRO DE DISPOSITIVO
        # ═══════════════════════════════════════════════════════════
        self._registrar_dispositivo()
        
        # ═══════════════════════════════════════════════════════════
        # SUSCRIPTORES ROS 2
        # ═══════════════════════════════════════════════════════════
        self.sub_temp = self.create_subscription(
            Float32,
            '/temperatura',
            self._callback_temperatura,
            10
        )
        
        self.sub_ph = self.create_subscription(
            Float32,
            '/ph',
            self._callback_ph,
            10
        )
        
        # Timer para guardar periódicamente
        self.timer = self.create_timer(
            self.INTERVALO_GUARDADO,
            self._timer_guardar
        )
        
        self.get_logger().info("Nodo iniciado - Esperando datos...")
        self.get_logger().info(f"   Suscrito a: /temperatura, /ph")
        self.get_logger().info(f"   Intervalo de guardado: {self.INTERVALO_GUARDADO}s")
    
    def _registrar_dispositivo(self):
        """Registra el dispositivo si no existe."""
        resultado = self.db.registrar_dispositivo(
            dispositivo_id=self.DISPOSITIVO_ID,
            auto_registrado=True
        )
        if resultado:
            self.get_logger().info(f"Nuevo dispositivo registrado: {self.DISPOSITIVO_ID}")
        else:
            self.get_logger().info(f"Dispositivo existente: {self.DISPOSITIVO_ID}")
    
    def _callback_temperatura(self, msg: Float32):
        """Callback cuando llega un mensaje de temperatura."""
        self._temperatura_actual = msg.data
        self.get_logger().debug(f"Temperatura recibida: {msg.data}°C")
    
    def _callback_ph(self, msg: Float32):
        """Callback cuando llega un mensaje de pH."""
        self._ph_actual = msg.data
        self.get_logger().debug(f"pH recibido: {msg.data}")
    
    def _timer_guardar(self):
        """Timer que guarda periódicamente los valores actuales."""
        # Solo guardar si hay al menos un valor
        if self._temperatura_actual is None and self._ph_actual is None:
            return
        
        try:
            # Guardar lectura (la calibración se aplica automáticamente)
            doc_id = self.db.guardar_lectura(
                dispositivo_id=self.DISPOSITIVO_ID,
                temperatura=self._temperatura_actual,
                ph=self._ph_actual
            )
            
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.get_logger().info(
                f"[{timestamp}] Guardado: "
                f"T={self._temperatura_actual}°C, pH={self._ph_actual}"
            )
            
            self._ultimo_guardado = datetime.now()
            
        except Exception as e:
            self.get_logger().error(f"[ERROR] Error guardando: {e}")
    
    def destroy_node(self):
        """Limpieza al cerrar el nodo."""
        if hasattr(self, 'db'):
            self.db.cerrar()
            self.get_logger().info("Conexión a MongoDB cerrada")
        super().destroy_node()


def main(args=None):
    """Función principal."""
    print("=" * 60)
    print("   NODO ROS 2 - SENSOR DB")
    print("=" * 60)
    
    rclpy.init(args=args)
    
    try:
        node = SensorDBNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nDetenido por el usuario")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
