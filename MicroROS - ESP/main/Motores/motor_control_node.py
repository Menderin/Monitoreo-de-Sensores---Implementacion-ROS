#!/usr/bin/env python3
"""
Nodo ROS 2 para control de motores DC con teclado y velocidad variable.

Este nodo:
- Publica comandos de motor al topic /motor_commands
- Captura teclas A/D (dirección) y 1/2/3 (velocidad) desde el teclado
- Independiente del sistema de sensores

Controles:
    A - Izquierda
    D - Derecha
    S - Detener
    1 - Velocidad Lenta (40%)
    2 - Velocidad Media (70%)
    3 - Velocidad Rápida (100%)
    Q - Salir

Uso:
    python3 motor_control_node.py
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Importar para keyboard input (Unix/Linux)
try:
    import termios
    import tty
    KEYBOARD_AVAILABLE = True
except ImportError:
    KEYBOARD_AVAILABLE = False


class MotorControlNode(Node):
    """
    Nodo ROS 2 dedicado al control de motores via teclado con velocidad variable.
    """
    
    def __init__(self):
        super().__init__('motor_control_node')
        
        # ═══════════════════════════════════════════════════════════
        # PUBLICADOR DE COMANDOS
        # ═══════════════════════════════════════════════════════════
        self.pub_motor_commands = self.create_publisher(
            String,
            '/motor_commands',
            10
        )
        
        # Estado del motor
        self.motor_state = "STOPPED"  # STOPPED, LEFT, RIGHT
        self.current_speed_percent = 100  # Velocidad inicial
        
        # ═══════════════════════════════════════════════════════════
        # CONTROL DE TECLADO
        # ═══════════════════════════════════════════════════════════
        self._keyboard_active = False
        if KEYBOARD_AVAILABLE:
            self._keyboard_active = True
            self._keyboard_thread = threading.Thread(
                target=self._keyboard_listener,
                daemon=True
            )
            self._keyboard_thread.start()
            self.get_logger().info("Teclado activado - Presiona A/D (dirección) y W/E (velocidad)")
        else:
            self.get_logger().warn("Módulo termios no disponible - Teclado deshabilitado")
        
        self.get_logger().info("Nodo de control de motor iniciado")
        self.get_logger().info(f"   Publicando a: /motor_commands (String)")
    
    def _getch(self):
        """Lee un solo carácter del teclado sin echo (Unix/Linux)."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def _publish_command(self, command):
        """Helper para publicar comandos."""
        msg = String()
        msg.data = command
        self.pub_motor_commands.publish(msg)
    
    def _keyboard_listener(self):
        """Thread que escucha el teclado y publica comandos de motor."""
        self.get_logger().info("")
        self.get_logger().info("═" * 60)
        self.get_logger().info("  CONTROL DE MOTORES CON VELOCIDAD VARIABLE")
        self.get_logger().info("═" * 60)
        self.get_logger().info("  Dirección:")
        self.get_logger().info("    A → Mover izquierda")
        self.get_logger().info("    D → Mover derecha")
        self.get_logger().info("    S → Detener motor")
        self.get_logger().info("")
        self.get_logger().info("  Velocidad (incrementos de 10%):")
        self.get_logger().info("    W → Disminuir velocidad (-10%)")
        self.get_logger().info("    E → Aumentar velocidad (+10%)")
        self.get_logger().info("    ⚡ Cambios se aplican inmediatamente si el motor está girando")
        self.get_logger().info("")
        self.get_logger().info("  Atajos:")
        self.get_logger().info("    1 → 🐌 Lenta (40%)")
        self.get_logger().info("    2 → 🚶 Media (70%)")
        self.get_logger().info("    3 → 🚀 Rápida (100%)")
        self.get_logger().info("")
        self.get_logger().info("    Q → Salir")
        self.get_logger().info("═" * 60)
        self.get_logger().info(f"  Velocidad actual: {self.current_speed_percent}%")
        self.get_logger().info(f"  Estado motor: {self.motor_state}")
        self.get_logger().info("═" * 60)
        self.get_logger().info("")
        
        while rclpy.ok() and self._keyboard_active:
            try:
                key = self._getch().lower()
                
                # Comandos de dirección
                if key == 'a':
                    self.motor_state = "LEFT"
                    self._publish_command("LEFT")
                    self.get_logger().info(f" IZQUIERDA [{self.current_speed_percent}%]")
                    
                elif key == 'd':
                    self.motor_state = "RIGHT"
                    self._publish_command("RIGHT")
                    self.get_logger().info(f" DERECHA [{self.current_speed_percent}%]")
                
                elif key == 's':
                    self.motor_state = "STOPPED"
                    self._publish_command("STOP")
                    self.get_logger().info("⏸  DETENIDO")
                
                # Control incremental de velocidad
                elif key == 'w':
                    # Disminuir 10%
                    self.current_speed_percent = max(10, self.current_speed_percent - 10)
                    self._publish_command(f"SPEED_SET_{self.current_speed_percent}")
                    self.get_logger().info(f"🔽 Velocidad: {self.current_speed_percent}%")
                    
                    # Si el motor está girando, reaplicar dirección con nueva velocidad
                    if self.motor_state == "LEFT":
                        self._publish_command("LEFT")
                        self.get_logger().info(f"   ↻ Aplicando a motor IZQUIERDA")
                    elif self.motor_state == "RIGHT":
                        self._publish_command("RIGHT")
                        self.get_logger().info(f"   ↻ Aplicando a motor DERECHA")
                
                elif key == 'e':
                    # Aumentar 10%
                    self.current_speed_percent = min(100, self.current_speed_percent + 10)
                    self._publish_command(f"SPEED_SET_{self.current_speed_percent}")
                    self.get_logger().info(f"🔼 Velocidad: {self.current_speed_percent}%")
                    
                    # Si el motor está girando, reaplicar dirección con nueva velocidad
                    if self.motor_state == "LEFT":
                        self._publish_command("LEFT")
                        self.get_logger().info(f"   ↻ Aplicando a motor IZQUIERDA")
                    elif self.motor_state == "RIGHT":
                        self._publish_command("RIGHT")
                        self.get_logger().info(f"   ↻ Aplicando a motor DERECHA")
                
                # Atajos de velocidad
                elif key == '1':
                    self.current_speed_percent = 40
                    self._publish_command("SPEED_SET_40")
                    self.get_logger().info(f"🎚️  Velocidad: {self.current_speed_percent}% (Lenta)")
                    
                    # Reaplicar si está en movimiento
                    if self.motor_state == "LEFT":
                        self._publish_command("LEFT")
                    elif self.motor_state == "RIGHT":
                        self._publish_command("RIGHT")
                
                elif key == '2':
                    self.current_speed_percent = 70
                    self._publish_command("SPEED_SET_70")
                    self.get_logger().info(f"🎚️  Velocidad: {self.current_speed_percent}% (Media)")
                    
                    # Reaplicar si está en movimiento
                    if self.motor_state == "LEFT":
                        self._publish_command("LEFT")
                    elif self.motor_state == "RIGHT":
                        self._publish_command("RIGHT")
                
                elif key == '3':
                    self.current_speed_percent = 100
                    self._publish_command("SPEED_SET_100")
                    self.get_logger().info(f"🎚️  Velocidad: {self.current_speed_percent}% (Rápida)")
                    
                    # Reaplicar si está en movimiento
                    if self.motor_state == "LEFT":
                        self._publish_command("LEFT")
                    elif self.motor_state == "RIGHT":
                        self._publish_command("RIGHT")
                    
                elif key == 'q':
                    self.get_logger().info("Deteniendo listener de teclado...")
                    self._keyboard_active = False
                    break
                    
            except Exception as e:
                self.get_logger().error(f"Error en keyboard listener: {e}")
                break


def main(args=None):
    """Función principal."""
    print("=" * 60)
    print("   NODO ROS 2 - CONTROL DE MOTOR CON VELOCIDAD")
    print("=" * 60)
    
    if not KEYBOARD_AVAILABLE:
        print("ERROR: Módulo termios no disponible")
        print("Este nodo requiere un sistema Unix/Linux")
        return
    
    rclpy.init(args=args)
    
    try:
        node = MotorControlNode()
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
