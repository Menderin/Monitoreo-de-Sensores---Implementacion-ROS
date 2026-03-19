#!/usr/bin/env python3
"""
Nodo ROS 2 — Control dual de motores DC con teclado.

Controles:
    M / Tab → Cambiar motor activo (M1 ↔ M2)
    A       → Izquierda
    D       → Derecha
    S       → Detener
    W       → Velocidad -10%
    E       → Velocidad +10%
    1       → 40%  (Lenta)
    2       → 70%  (Media)
    3       → 100% (Rápida)
    I       → Mostrar estado
    Q       → Salir

Comandos publicados en /motor_commands (String):
    LEFT | RIGHT | STOP
    SPEED_SET_<n>        (n = porcentaje 0-100)
    SELECT_MOTOR_<n>     (n = 1 o 2)
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import termios
    import tty
    KEYBOARD_AVAILABLE = True
except ImportError:
    KEYBOARD_AVAILABLE = False

MOTOR_LABELS = {1: "MOTOR 1  (GPIO 25/26/14)",
                2: "MOTOR 2  (GPIO 27/33/32)"}


def _indicator(active: int) -> str:
    return ("[ M1 ] <── activo     [ M2 ]" if active == 1
            else "[ M1 ]     activo ──> [ M2 ]")


class MotorControlNode(Node):

    def __init__(self):
        super().__init__('motor_control_node')

        self.pub = self.create_publisher(String, '/motor_commands', 10)

        self.motor_state   = "STOPPED"
        self.speed_pct     = 100
        self.active_motor  = 1          # 1 o 2

        self._running = False
        if KEYBOARD_AVAILABLE:
            self._running = True
            threading.Thread(target=self._loop, daemon=True).start()
            self.get_logger().info("Teclado activo")
        else:
            self.get_logger().warn("termios no disponible — teclado deshabilitado")

        self.get_logger().info("Publicando en /motor_commands")

    # ── helpers ─────────────────────────────────────────────

    def _getch(self) -> str:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _pub(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)

    def _log_indicator(self):
        self.get_logger().info("")
        self.get_logger().info("  ┌──────────────────────────────────────┐")
        self.get_logger().info(f"  │  Motor activo: {MOTOR_LABELS[self.active_motor]:<22}│")
        self.get_logger().info(f"  │  {_indicator(self.active_motor):<36}│")
        self.get_logger().info("  └──────────────────────────────────────┘")
        self.get_logger().info("")

    def _log_header(self):
        self.get_logger().info("")
        self.get_logger().info("═" * 55)
        self.get_logger().info("  CONTROL DUAL DE MOTORES")
        self.get_logger().info("═" * 55)
        self.get_logger().info("  M / Tab  → Cambiar motor activo (M1 ↔ M2)")
        self.get_logger().info("  A / D    → Izquierda / Derecha")
        self.get_logger().info("  S        → Detener")
        self.get_logger().info("  W / E    → Velocidad  -10% / +10%")
        self.get_logger().info("  1/2/3    → 40% / 70% / 100%")
        self.get_logger().info("  I        → Estado    Q → Salir")
        self.get_logger().info("═" * 55)
        self.get_logger().info(
            f"  Motor activo : {MOTOR_LABELS[self.active_motor]}")
        self.get_logger().info(f"  Velocidad    : {self.speed_pct}%")
        self.get_logger().info(f"  Estado       : {self.motor_state}")
        self.get_logger().info("═" * 55)
        self.get_logger().info("")

    def _switch_motor(self):
        self.active_motor = 2 if self.active_motor == 1 else 1
        self._pub(f"SELECT_MOTOR_{self.active_motor}")
        self._log_indicator()

    def _reapply(self):
        """Reaplicar dirección al motor activo tras cambiar velocidad."""
        if self.motor_state == "LEFT":
            self._pub("LEFT")
        elif self.motor_state == "RIGHT":
            self._pub("RIGHT")

    # ── loop ────────────────────────────────────────────────

    def _loop(self):
        self._log_header()

        while rclpy.ok() and self._running:
            try:
                key = self._getch().lower()
            except Exception as e:
                self.get_logger().error(f"Error teclado: {e}")
                break

            # Selección de motor
            if key in ('m', '\t'):
                self._switch_motor()

            # Dirección
            elif key == 'a':
                self.motor_state = "LEFT"
                self._pub("LEFT")
                self.get_logger().info(
                    f"  [M{self.active_motor}] <-- IZQUIERDA  [{self.speed_pct}%]")

            elif key == 'd':
                self.motor_state = "RIGHT"
                self._pub("RIGHT")
                self.get_logger().info(
                    f"  [M{self.active_motor}] --> DERECHA    [{self.speed_pct}%]")

            elif key == 's':
                self.motor_state = "STOPPED"
                self._pub("STOP")
                self.get_logger().info(f"  [M{self.active_motor}] || DETENIDO")

            # Velocidad incremental
            elif key == 'w':
                self.speed_pct = max(10, self.speed_pct - 10)
                self._pub(f"SPEED_SET_{self.speed_pct}")
                self.get_logger().info(f"  Velocidad: {self.speed_pct}%  (-10%)")
                self._reapply()

            elif key == 'e':
                self.speed_pct = min(100, self.speed_pct + 10)
                self._pub(f"SPEED_SET_{self.speed_pct}")
                self.get_logger().info(f"  Velocidad: {self.speed_pct}%  (+10%)")
                self._reapply()

            # Atajos de velocidad
            elif key == '1':
                self.speed_pct = 40
                self._pub("SPEED_SET_40")
                self.get_logger().info("  Velocidad: 40%  (Lenta)")
                self._reapply()

            elif key == '2':
                self.speed_pct = 70
                self._pub("SPEED_SET_70")
                self.get_logger().info("  Velocidad: 70%  (Media)")
                self._reapply()

            elif key == '3':
                self.speed_pct = 100
                self._pub("SPEED_SET_100")
                self.get_logger().info("  Velocidad: 100% (Rápida)")
                self._reapply()

            # Info / salir
            elif key == 'i':
                self._log_header()

            elif key == 'q':
                self.get_logger().info("Saliendo...")
                self._running = False
                break


def main(args=None):
    print("=" * 55)
    print("   NODO ROS 2 — CONTROL DUAL DE MOTORES")
    print("   M1: GPIO 25 / 26 / 14")
    print("   M2: GPIO 27 / 33 / 32")
    print("=" * 55)

    if not KEYBOARD_AVAILABLE:
        print("ERROR: termios no disponible (requiere Linux)")
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
