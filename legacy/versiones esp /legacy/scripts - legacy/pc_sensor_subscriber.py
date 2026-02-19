#!/usr/bin/env python3
"""
Nodo ROS 2 suscriptor de temperatura y pH desde ESP32
Este nodo se ejecuta en el PC y recibe datos del ESP32 con sensor CWT-BL
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from datetime import datetime


class SensorSubscriber(Node):
    """
    Nodo que se suscribe a los tópicos /temperatura y /ph publicados por el ESP32
    """

    def __init__(self):
        super().__init__('pc_sensor_subscriber')
        
        # Crear suscriptor al tópico /temperatura
        self.temp_subscription = self.create_subscription(
            Float32,
            'temperatura',
            self.temperature_callback,
            10)  # QoS: buffer de 10 mensajes
        
        # Crear suscriptor al tópico /ph
        self.ph_subscription = self.create_subscription(
            Float32,
            'ph',
            self.ph_callback,
            10)  # QoS: buffer de 10 mensajes
        
        # Variables para temperatura
        self.temp_count = 0
        self.temp_sum = 0.0
        self.temp_min = float('inf')
        self.temp_max = float('-inf')
        self.last_temp = None
        
        # Variables para pH
        self.ph_count = 0
        self.ph_sum = 0.0
        self.ph_min = float('inf')
        self.ph_max = float('-inf')
        self.last_ph = None
        
        # Umbrales de alerta
        self.temp_warning_high = 30.0  # °C
        self.temp_warning_low = 15.0   # °C
        self.ph_warning_high = 8.5     # pH alcalino
        self.ph_warning_low = 6.5      # pH ácido
        
        self.get_logger().info('🖥️  Nodo PC iniciado: Esperando datos del ESP32...')
        self.get_logger().info(f'📊 Alertas Temperatura: <{self.temp_warning_low}°C o >{self.temp_warning_high}°C')
        self.get_logger().info(f'📊 Alertas pH: <{self.ph_warning_low} o >{self.ph_warning_high}')
        self.get_logger().info('='*70)

    def temperature_callback(self, msg):
        """
        Callback ejecutado cada vez que llega un mensaje de temperatura
        """
        temp_c = msg.data
        self.last_temp = temp_c
        
        # Actualizar estadísticas
        self.temp_count += 1
        self.temp_sum += temp_c
        self.temp_min = min(self.temp_min, temp_c)
        self.temp_max = max(self.temp_max, temp_c)
        temp_avg = self.temp_sum / self.temp_count
        
        # Timestamp
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Detectar alertas
        alert = ""
        if temp_c > self.temp_warning_high:
            alert = " ⚠️  ALTA"
        elif temp_c < self.temp_warning_low:
            alert = " ⚠️  BAJA"
        
        # Mostrar datos
        self.get_logger().info(
            f'[{timestamp}] 🌡️  Temperatura: {temp_c:.2f}°C | '
            f'Promedio: {temp_avg:.2f}°C | Min: {self.temp_min:.2f}°C | Max: {self.temp_max:.2f}°C{alert}'
        )
        
        # Mostrar estado completo si ambos sensores han reportado
        self.print_combined_status()

    def ph_callback(self, msg):
        """
        Callback ejecutado cada vez que llega un mensaje de pH
        """
        ph_value = msg.data
        self.last_ph = ph_value
        
        # Actualizar estadísticas
        self.ph_count += 1
        self.ph_sum += ph_value
        self.ph_min = min(self.ph_min, ph_value)
        self.ph_max = max(self.ph_max, ph_value)
        ph_avg = self.ph_sum / self.ph_count
        
        # Timestamp
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Detectar alertas y clasificación
        alert = ""
        classification = ""
        
        if ph_value < 6.5:
            classification = "ÁCIDO"
            if ph_value < self.ph_warning_low:
                alert = " ⚠️"
        elif ph_value > 8.5:
            classification = "ALCALINO"
            if ph_value > self.ph_warning_high:
                alert = " ⚠️"
        else:
            classification = "NEUTRO"
        
        # Mostrar datos
        self.get_logger().info(
            f'[{timestamp}] 🧪 pH: {ph_value:.2f} [{classification}] | '
            f'Promedio: {ph_avg:.2f} | Min: {self.ph_min:.2f} | Max: {self.ph_max:.2f}{alert}'
        )
        
        # Mostrar estado completo si ambos sensores han reportado
        self.print_combined_status()

    def print_combined_status(self):
        """
        Imprime un resumen combinado cuando ambos sensores tienen datos
        """
        if self.last_temp is not None and self.last_ph is not None:
            self.get_logger().info(
                f'📊 Estado actual: Temp={self.last_temp:.2f}°C | pH={self.last_ph:.2f}'
            )
            self.get_logger().info('-'*70)


def main(args=None):
    """
    Función principal
    """
    rclpy.init(args=args)
    
    sensor_subscriber = SensorSubscriber()
    
    try:
        rclpy.spin(sensor_subscriber)
    except KeyboardInterrupt:
        print('\n🛑 Detenido por el usuario')
    finally:
        # Mostrar resumen final
        if sensor_subscriber.temp_count > 0:
            print('\n' + '='*70)
            print('📈 RESUMEN DE TEMPERATURA:')
            print(f'   Total de lecturas: {sensor_subscriber.temp_count}')
            print(f'   Promedio: {sensor_subscriber.temp_sum/sensor_subscriber.temp_count:.2f}°C')
            print(f'   Mínima: {sensor_subscriber.temp_min:.2f}°C')
            print(f'   Máxima: {sensor_subscriber.temp_max:.2f}°C')
        
        if sensor_subscriber.ph_count > 0:
            print('\n📈 RESUMEN DE pH:')
            print(f'   Total de lecturas: {sensor_subscriber.ph_count}')
            print(f'   Promedio: {sensor_subscriber.ph_sum/sensor_subscriber.ph_count:.2f}')
            print(f'   Mínimo: {sensor_subscriber.ph_min:.2f}')
            print(f'   Máximo: {sensor_subscriber.ph_max:.2f}')
            print('='*70)
        
        sensor_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
