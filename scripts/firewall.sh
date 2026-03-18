#!/bin/bash
# ==============================================================================
# firewall.sh — Gateway Monitoring System (Raspberry Pi 4)
# Configura reglas iptables: modo Gateway Aislado
#
# Interfaces por defecto:
#   IF_WAN  — Ethernet (Internet / Red Lab)   → 192.168.0.x
#   IF_LAN  — WiFi Hotspot (Red sensores)     → 10.42.0.1
#
# Sobreescribirlas sin editar el script:
#   sudo IF_WAN=eth1 IF_LAN=wlan1 ./firewall.sh
# ==============================================================================


# ── Auto-detección de interfaces (sobreescribible por entorno) ────────────────
# Si ya fueron exportadas externamente, se respetan.
if [[ -z "${IF_WAN:-}" ]]; then
    # Primera interfaz Ethernet activa (empieza con 'e': eth0, enp3s0, etc.)
    IF_WAN=$(ip -o link show up | awk -F': ' '$2~/^e/{print $2; exit}')
fi
if [[ -z "${IF_LAN:-}" ]]; then
    # Primera interfaz WiFi activa (empieza con 'w': wlan0, wlp2s0, etc.)
    IF_LAN=$(ip -o link show up | awk -F': ' '$2~/^w/{print $2; exit}')
fi
IF_WAN="${IF_WAN:-eth0}"
IF_LAN="${IF_LAN:-wlan0}"

# ── Mostrar y confirmar interfaces ───────────────────────────────────────────
echo ""
echo "======================================================"
echo "  INTERFACES DETECTADAS"
echo "======================================================"
echo "  IF_WAN (Internet/Lab) : $IF_WAN"
echo "  IF_LAN (Sensores WiFi): $IF_LAN"
echo "======================================================"
echo ""
read -rp "¿Las interfaces son correctas? [S/n]: " _confirm_if
echo ""
if [[ "$_confirm_if" =~ ^[nN]$ ]]; then
    read -rp "  Ingresa la interfaz WAN (ej: eth0, enp3s0): " _input_wan
    read -rp "  Ingresa la interfaz LAN (ej: wlan0, wlp2s0): " _input_lan
    [[ -n "$_input_wan" ]] && IF_WAN="$_input_wan"
    [[ -n "$_input_lan" ]] && IF_LAN="$_input_lan"
    echo ""
    echo "[INFO] Usando: WAN=$IF_WAN  LAN=$IF_LAN"
    echo ""
fi


# --- 1. Limpieza total ---
echo "[*] Limpiando reglas antiguas..."
sudo iptables -P INPUT ACCEPT
sudo iptables -P FORWARD ACCEPT
sudo iptables -P OUTPUT ACCEPT
sudo iptables -F
sudo iptables -X
sudo iptables -t nat -F
sudo iptables -t nat -X

# --- 2. Políticas restrictivas (Seguridad Positiva) ---
echo "[*] Estableciendo políticas restrictivas..."
sudo iptables -P INPUT DROP
sudo iptables -P FORWARD DROP   # Robots NO tienen acceso a Internet
sudo iptables -P OUTPUT ACCEPT

# --- 3. Reglas del sistema (Raspberry Pi) ---
# Loopback: crítico para ROS 2 (DDS usa localhost)
sudo iptables -A INPUT -i lo -j ACCEPT

# Stateful inspection: solo respuestas a conexiones iniciadas por la Pi
sudo iptables -A INPUT -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT

# SSH desde la LAN cableada (administración)
sudo iptables -A INPUT -i "$IF_WAN" -p tcp --dport 22 -j ACCEPT

# --- 4. Red de Sensores (hotspot WiFi, sensores ESP32) ---
echo "[*] Configurando permisos para Red de Sensores ($IF_LAN)..."

# DHCP & DNS: los ESP32 necesitan IP y resolución de nombres
sudo iptables -A INPUT -i "$IF_LAN" -p udp --dport 67:68 --sport 67:68 -j ACCEPT
sudo iptables -A INPUT -i "$IF_LAN" -p udp --dport 53 -j ACCEPT
sudo iptables -A INPUT -i "$IF_LAN" -p tcp --dport 53 -j ACCEPT

# micro-ROS UDP: el Agent escucha en UDP 8888; solo permite ese puerto
sudo iptables -A INPUT -i "$IF_LAN" -p udp --dport 8888 -j ACCEPT

# Ping desde sensores: diagnóstico básico
sudo iptables -A INPUT -i "$IF_LAN" -p icmp -j ACCEPT

# Bloqueo explícito: SSH desde los robots (hardening)
sudo iptables -A INPUT -i "$IF_LAN" -p tcp --dport 22 -j DROP

# --- 5. Guardar reglas (persistencia) ---
if ! dpkg -l iptables-persistent &>/dev/null 2>&1; then
    echo "[*] Instalando iptables-persistent para guardar reglas..."
    # DEBIAN_FRONTEND=noninteractive evita el diálogo interactivo de instalación
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y iptables-persistent -qq
fi

echo "[*] Guardando reglas permanentemente con netfilter-persistent..."
sudo netfilter-persistent save
echo "[*] Reglas guardadas. Sobrevivirán al próximo reinicio."

# --- Resultado ---
echo "==================================================="
echo " ✅ FIREWALL APLICADO: MODO GATEWAY AISLADO"
echo "==================================================="
echo " WAN ($IF_WAN): SSH permitido. Internet para la Pi."
echo " LAN ($IF_LAN): Solo DHCP, DNS y micro-ROS (UDP)."
echo " AISLAMIENTO: Forwarding DESACTIVADO (ESP32 sin internet)."
echo "==================================================="
