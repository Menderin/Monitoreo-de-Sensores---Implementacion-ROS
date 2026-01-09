#!/usr/bin/env python3
"""
Nodo ROS 2 suscriptor de temperatura desde ESP32
Este nodo se ejecuta en el PC y recibe datos del ESP32
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from datetime import datetime


class TemperatureSubscriber(Node):
    """
    Nodo que se suscribe al tópico /temperatura publicado por el ESP32
    """

    def __init__(self):
        super().__init__('pc_temp_subscriber')
        
        # Crear suscriptor al tópico /temperatura
        self.subscription = self.create_subscription(
            Float32,
            'temperatura',
            self.temperature_callback,
            10)  # QoS: buffer de 10 mensajes
        
        # Variables para estadísticas
        self.count = 0
        self.temp_sum = 0.0
        self.temp_min = float('inf')
        self.temp_max = float('-inf')
        
        # Umbrales de alerta
        self.temp_warning_high = 30.0  # °C
        self.temp_warning_low = 15.0   # °C
        
        self.get_logger().info('🖥️  Nodo PC iniciado: Esperando datos del ESP32...')
        self.get_logger().info(f'📊 Alertas configuradas: <{self.temp_warning_low}°C o >{self.temp_warning_high}°C')

    def temperature_callback(self, msg):
        """
        Callback ejecutado cada vez que llega un mensaje de temperatura
        """
        temp_c = msg.data
        temp_f = (temp_c * 9/5) + 32  # Convertir a Fahrenheit
        
        # Actualizar estadísticas
        self.count += 1
        self.temp_sum += temp_c
        self.temp_min = min(self.temp_min, temp_c)
        self.temp_max = max(self.temp_max, temp_c)
        temp_avg = self.temp_sum / self.count
        
        # Timestamp
        timestamp = datetime.now().strftime('%H:%M:%S')
        
        # Determinar estado
        emoji = '🌡️'
        status = 'Normal'
        if temp_c > self.temp_warning_high:
            emoji = '🔥'
            status = 'ALTA'
        elif temp_c < self.temp_warning_low:
            emoji = '❄️'
            status = 'BAJA'
        
        # Log principal
        self.get_logger().info(
            f'{emoji} [{timestamp}] Temp: {temp_c:.2f}°C ({temp_f:.2f}°F) | '
            f'Estado: {status} | #Lectura: {self.count}'
        )
        
        # Estadísticas cada 10 lecturas
        if self.count % 10 == 0:
            self.get_logger().info(
                f'📊 Estadísticas (últimas {self.count} lecturas): '
                f'Promedio={temp_avg:.2f}°C | Min={self.temp_min:.2f}°C | Max={self.temp_max:.2f}°C'
            )


def main(args=None):
    # Inicializar ROS 2
    rclpy.init(args=args)
    
    # Crear nodo
    temperature_subscriber = TemperatureSubscriber()
    
    try:
        # Mantener el nodo ejecutándose
        rclpy.spin(temperature_subscriber)
    except KeyboardInterrupt:
        print('\n⛔ Detenido por el usuario')
    finally:
        # Limpieza
        temperature_subscriber.get_logger().info('👋 Cerrando nodo PC...')
        temperature_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
