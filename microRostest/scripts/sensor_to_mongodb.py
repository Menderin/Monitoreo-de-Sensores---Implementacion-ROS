#!/usr/bin/env python3
"""
Nodo ROS 2 que recibe datos de sensores y los envía a MongoDB
Suscribe a /temperatura y /ph, crea documentos JSON y los almacena
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from datetime import datetime
import json
import os
from pathlib import Path

# Cargar variables de entorno desde archivo .env
from dotenv import load_dotenv
load_dotenv()

# Descomentar cuando instales pymongo
from pymongo import MongoClient
from pymongo.errors import ConnectionFailure


class SensorToMongoNode(Node):
    """
    Nodo que recibe datos de sensores y los guarda en MongoDB
    """

    def __init__(self):
        super().__init__('sensor_to_mongodb')
        
        # Variables para almacenar últimos valores
        self.last_temperature = None
        self.last_ph = None
        self.last_temp_time = None
        self.last_ph_time = None
        
        # Contador de documentos guardados
        self.documents_saved = 0
        
        # Configuración MongoDB desde variables de entorno (.env)
        self.mongo_uri = os.getenv('MONGO_URI')
        self.db_name = os.getenv('MONGO_DB', 'Datos_ESP')
        self.collection_name = os.getenv('MONGO_COLLECTION', 'datos_sensores')
        
        # Validar que se haya configurado la URI
        if not self.mongo_uri:
            self.get_logger().error('❌ MONGO_URI no configurado en archivo .env')
            self.get_logger().error('💡 Copia .env.example a .env y configura tus credenciales')
            raise ValueError('MONGO_URI requerido en archivo .env')
        
        # Inicializar conexión MongoDB
        self.init_mongodb()
        
        # Crear suscriptores
        self.temp_subscription = self.create_subscription(
            Float32,
            'temperatura',
            self.temperature_callback,
            10)
        
        self.ph_subscription = self.create_subscription(
            Float32,
            'ph',
            self.ph_callback,
            10)
        
        # Timer para guardar datos cada X segundos (opcional - batch)
        # self.timer = self.create_timer(5.0, self.save_batch_data)
        
        self.get_logger().info('🗄️  Nodo MongoDB iniciado')
        self.get_logger().info(f'📊 Base de datos: {self.db_name}.{self.collection_name}')
        self.get_logger().info('='*70)

    def init_mongodb(self):
        """
        Inicializa la conexión a MongoDB
        """
        try:
            self.mongo_client = MongoClient(self.mongo_uri, serverSelectionTimeoutMS=5000)
            self.mongo_client.admin.command('ping')
            self.db = self.mongo_client[self.db_name]
            self.collection = self.db[self.collection_name]
            
            # Ocultar credenciales en el log
            uri_display = self.mongo_uri.split('@')[1] if '@' in self.mongo_uri else self.mongo_uri
            self.get_logger().info(f'✅ Conectado a MongoDB Atlas: {uri_display}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error conectando a MongoDB: {e}')
            self.mongo_client = None
            self.collection = None

    def temperature_callback(self, msg):
        """
        Callback para temperatura
        """
        self.last_temperature = msg.data
        self.last_temp_time = datetime.now()
        # No guardar aquí, esperar al callback de pH

    def ph_callback(self, msg):
        """
        Callback para pH
        """
        self.last_ph = msg.data
        self.last_ph_time = datetime.now()
        
        # Guardar inmediatamente si tenemos ambos valores
        if self.last_temperature is not None:
            self.save_sensor_data()

    def save_sensor_data(self):
        """
        Guarda los datos de sensores en MongoDB como documento JSON
        """
        if self.last_temperature is None or self.last_ph is None:
            return
        
        # Crear documento JSON simplificado
        timestamp = datetime.now()
        document = {
            'timestamp': timestamp.isoformat(),
            'temperatura': {
                'valor': round(self.last_temperature, 2),
                'unidad': '°C'
            },
            'ph': {
                'valor': round(self.last_ph, 2),
                'unidad': 'pH'
            }
        }
        
        # Guardar en MongoDB
        if self.collection is not None:
            try:
                result = self.collection.insert_one(document)
                self.documents_saved += 1
                self.get_logger().info(
                    f'✅ Documento guardado (ID: {result.inserted_id}) | '
                    f'Temp: {self.last_temperature:.2f}°C | pH: {self.last_ph:.2f}'
                )
            except Exception as e:
                self.get_logger().error(f'❌ Error guardando en MongoDB: {e}')
        else:
            # Modo simulación: solo mostrar JSON
            self.documents_saved += 1
            json_str = json.dumps(document, indent=2, ensure_ascii=False)
            self.get_logger().info(f'📄 Documento #{self.documents_saved}:')
            self.get_logger().info(f'\n{json_str}')
            self.get_logger().info('-'*70)
        
        # Opcional: Guardar en archivo JSON local como respaldo
        self.save_to_json_file(document)

    def save_to_json_file(self, document):
        """
        Guarda el documento en un archivo JSON local como respaldo
        """
        try:
            # Crear carpeta de datos si no existe
            data_dir = os.path.join(
                os.path.dirname(__file__), 
                'datos_sensores'
            )
            os.makedirs(data_dir, exist_ok=True)
            
            # Nombre de archivo con fecha
            date_str = datetime.now().strftime('%Y-%m-%d')
            filename = os.path.join(data_dir, f'sensores_{date_str}.jsonl')
            
            # Crear copia del documento sin ObjectId para serialización
            doc_copy = {k: v for k, v in document.items() if k != '_id'}
            
            # Guardar en formato JSON Lines (un JSON por línea)
            with open(filename, 'a') as f:
                json.dump(doc_copy, f, ensure_ascii=False)
                f.write('\n')
                
        except Exception as e:
            self.get_logger().error(f'⚠️  Error guardando archivo local: {e}')

    def save_batch_data(self):
        """
        Guarda datos en batch (si usas timer)
        """
        if self.last_temperature is not None and self.last_ph is not None:
            self.save_sensor_data()


def main(args=None):
    """
    Función principal
    """
    rclpy.init(args=args)
    
    node = SensorToMongoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n🛑 Detenido por el usuario')
    finally:
        # Mostrar resumen
        print('\n' + '='*70)
        print(f'📊 RESUMEN:')
        print(f'   Total documentos guardados: {node.documents_saved}')
        
        if node.mongo_client:
            node.mongo_client.close()
            print('   Conexión MongoDB cerrada')
        
        print('='*70)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
