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
from std_msgs.msg import Float32MultiArray
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
        
        # Diccionario de dispositivos: {mac: {temp, ph, timestamp}}
        self._dispositivos = {}
        self._dispositivos_registrados = set()  # MACs ya registradas
        self.db = None  # Puede ser None si MongoDB no está disponible
        self._db_available = False
        
        # ═══════════════════════════════════════════════════════════
        # CONEXIÓN A BASE DE DATOS (con reintentos)
        # ═══════════════════════════════════════════════════════════
        self._conectar_mongodb()
        
        # ═══════════════════════════════════════════════════════════
        # SUSCRIPTORES ROS 2
        # ═══════════════════════════════════════════════════════════
        # Suscriptor único para array de datos con MAC embebida
        self.sub_sensor_data = self.create_subscription(
            Float32MultiArray,
            '/sensor_data',
            self._callback_sensor_data,
            10
        )
        
        # Timer para guardar periódicamente
        self.timer = self.create_timer(
            self.INTERVALO_GUARDADO,
            self._timer_guardar
        )
        
        self.get_logger().info("Nodo iniciado - Esperando datos...")
        self.get_logger().info(f"   Suscrito a: /sensor_data (Float32MultiArray)")
        self.get_logger().info(f"   Intervalo de guardado: {self.INTERVALO_GUARDADO}s")
    
    def _conectar_mongodb(self, reintentos=3):
        """Intenta conectar a MongoDB con reintentos automáticos."""
        self.get_logger().info("Conectando a MongoDB...")
        
        for intento in range(1, reintentos + 1):
            try:
                self.db = SensorDBService()
                if self.db.ping():
                    self._db_available = True
                    self.get_logger().info("[OK] Conectado a MongoDB Atlas")
                    info = MongoConfig.get_info()
                    self.get_logger().info(f"   Base de datos: {info['database']}")
                    self.get_logger().info(f"   Colección datos: {info['col_datos']}")
                    return
            except Exception as e:
                self.get_logger().warn(f"Intento {intento}/{reintentos} falló: {e}")
                if intento < reintentos:
                    import time
                    time.sleep(2)  # Esperar 2s entre reintentos
        
        # Si todos los intentos fallaron
        self.get_logger().warn("⚠️  MongoDB no disponible - Modo degradado activado")
        self.get_logger().warn("   El nodo recibirá datos pero NO los guardará")
        self._db_available = False
    
    def _registrar_dispositivo(self, mac):
        """Registra un dispositivo si no existe."""
        if mac in self._dispositivos_registrados:
            return  # Ya registrado
        
        if not self._db_available:
            self.get_logger().info(f"MAC detectada: {mac} (no guardado - DB offline)")
            self._dispositivos_registrados.add(mac)
            return
        
        resultado = self.db.registrar_dispositivo(
            dispositivo_id=mac,
            auto_registrado=True
        )
        if resultado:
            self.get_logger().info(f"Nuevo dispositivo registrado: {mac}")
        else:
            self.get_logger().info(f"Dispositivo existente: {mac}")
        
        self._dispositivos_registrados.add(mac)
    
    def _callback_sensor_data(self, msg: Float32MultiArray):
        """
        Callback para Float32MultiArray: [temp, pH, voltage, mac_part1, mac_part2]
        Almacena datos en diccionario por MAC.
        """
        try:
            if len(msg.data) >= 5:
                # Extraer valores
                temp = msg.data[0]
                ph = msg.data[1]
                # voltage_raw = msg.data[2]  # Por ahora no se usa
                
                # Reconstruir MAC desde 2 floats
                mac_part1 = int(msg.data[3])
                mac_part2 = int(msg.data[4])
                mac_str = f"{mac_part1:06X}{mac_part2:06X}"
                
                # Almacenar datos en diccionario
                self._dispositivos[mac_str] = {
                    'temperatura': temp,
                    'ph': ph,
                    'timestamp': self.get_clock().now().nanoseconds / 1e9
                }
                
                # Registrar dispositivo si es nuevo
                if mac_str not in self._dispositivos_registrados:
                    self.get_logger().info(f"MAC detectada: {mac_str}")
                    self._registrar_dispositivo(mac_str)
                
                self.get_logger().debug(
                    f"[{mac_str}] T={temp:.1f}°C | pH={ph:.2f}"
                )
            else:
                self.get_logger().warn(f"Array incompleto: {len(msg.data)} elementos (esperado: 5)")
                
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Error parseando array: {e}")
    
    def _timer_guardar(self):
        """Timer que guarda periódicamente los valores de todos los dispositivos."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Iterar sobre todos los dispositivos
        for mac, datos in list(self._dispositivos.items()):
            # Solo guardar si los datos son recientes (< 10s)
            edad_datos = current_time - datos['timestamp']
            
            if edad_datos < 10.0:
                temp = datos['temperatura']
                ph = datos['ph']
                
                # Guardar en MongoDB solo si está disponible
                if not self._db_available:
                    self.get_logger().debug(
                        f"[{timestamp}] [{mac}] T={temp:.1f}°C, pH={ph:.2f} (DB offline)"
                    )
                    continue
                
                try:
                    self.db.guardar_lectura(
                        dispositivo_id=mac,
                        temperatura=temp,
                        ph=ph
                    )
                    self.get_logger().info(
                        f"[{timestamp}] [{mac}] T={temp:.1f}°C, pH={ph:.2f}"
                    )
                except Exception as e:
                    self.get_logger().error(f"Error guardando datos de {mac}: {e}")
            else:
                # Datos obsoletos, advertir
                self.get_logger().warn(
                    f"Dispositivo {mac} sin datos frescos ({edad_datos:.0f}s)"
                )
    
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
