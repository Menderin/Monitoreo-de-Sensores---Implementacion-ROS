#!/bin/bash
# ==============================================================================
# status.sh — Gateway Monitoring System 2.0
# Panel de diagnóstico completo: red, servicios, hardware, almacenamiento y logs.
# Puede ejecutarse directamente o ser llamado desde menu.sh (opción 4 → a).
# ==============================================================================

G='\033[0;32m'; R='\033[0;31m'; B='\033[0;34m'; Y='\033[1;33m'; NC='\033[0m'; BOLD='\033[1m'

# Interfaz WiFi hotspot: sobreescribible por entorno
HOTSPOT_IF="${HOTSPOT_IF:-wlan0}"

divider() { echo -e "${B}=================================================${NC}"; }

divider
echo -e "    📋  STATUS GATEWAY MONITORING SYSTEM 2.0    "
divider

# ── 1. Sistema ────────────────────────────────────────────
echo -e "\n${B}${BOLD}[ Sistema ]${NC}"

# Uptime
UPTIME=$(uptime -p 2>/dev/null || uptime 2>/dev/null | grep -oP 'up\s+\K[^,]+' || echo "n/a")
echo -e "  Uptime    : $UPTIME"

# Temperatura CPU (compatible con RPi + Ubuntu 24.04)
CPU_TEMP="n/a"
if [[ -f /sys/class/thermal/thermal_zone0/temp ]]; then
    raw=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 0)
    CPU_TEMP=$(awk "BEGIN {printf \"%.1f°C\", $raw/1000}")
elif command -v vcgencmd &>/dev/null; then
    CPU_TEMP=$(vcgencmd measure_temp 2>/dev/null | grep -oP '[\d.]+' | head -1)
    CPU_TEMP="${CPU_TEMP}°C"
fi
# Advertencia de temperatura alta (>75°C)
TEMP_VAL=$(echo "$CPU_TEMP" | grep -oP '[\d.]+' || echo 0)
if awk "BEGIN {exit !($TEMP_VAL > 75)}"; then
    echo -e "  CPU Temp  : ${R}${CPU_TEMP} ⚠ CALIENTE${NC}"
elif awk "BEGIN {exit !($TEMP_VAL > 60)}"; then
    echo -e "  CPU Temp  : ${Y}${CPU_TEMP}${NC}"
else
    echo -e "  CPU Temp  : ${G}${CPU_TEMP}${NC}"
fi

# Carga CPU
LOAD=$(uptime 2>/dev/null | grep -oP 'load average: \K[^,]+' || echo "n/a")
echo -e "  CPU Load  : $LOAD (1 min)"

# RAM
free -h 2>/dev/null | awk '/^Mem:/ {printf "  RAM       : %s usada / %s total\n", $3, $2}'

# ── 2. Almacenamiento (SD Card) ───────────────────────────
echo -e "\n${B}${BOLD}[ Almacenamiento ]${NC}"
df -h / 2>/dev/null | awk 'NR==2 {
    use = $5+0
    color = "\033[0;32m"
    warn  = ""
    if (use > 90) { color = "\033[0;31m"; warn = " ⚠ CRÍTICO" }
    else if (use > 75) { color = "\033[1;33m"; warn = " ⚠ ALTO" }
    printf "  Partición / : %s usados / %s total (%s\033[0m%s)\n", $3, $2, color $5, warn
}'

# ── 3. Red ────────────────────────────────────────────────
echo -e "\n${B}${BOLD}[ Red ]${NC}"
WAN_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo -e "  Ethernet (WAN) : ${WAN_IP:-n/a}"

WIFI_IP=$(ip addr show "$HOTSPOT_IF" 2>/dev/null | grep -oP 'inet \K[\d.]+' || true)
if [[ -n "$WIFI_IP" ]]; then
    echo -e "  WiFi Hotspot   : ${G}${WIFI_IP}${NC}  (${HOTSPOT_IF})"
else
    echo -e "  WiFi Hotspot   : ${R}OFF / no encontrado (${HOTSPOT_IF})${NC}"
fi

# ── 4. Servicios systemd ──────────────────────────────────
echo -e "\n${B}${BOLD}[ Servicios systemd ]${NC}"
for svc in smart-agent smart-bridge; do
    if systemctl is-active --quiet "$svc" 2>/dev/null; then
        # Tiempo desde que está activo
        SINCE=$(systemctl show "$svc" -p ActiveEnterTimestamp 2>/dev/null \
                | cut -d= -f2 \
                | xargs -I{} date -d "{}" "+%H:%M:%S del %d/%m" 2>/dev/null || true)
        echo -e "  ${svc}: ${G}ACTIVO${NC} ${SINCE:+(desde $SINCE)}"
    else
        STATE=$(systemctl is-active "$svc" 2>/dev/null || echo "inactivo")
        echo -e "  ${svc}: ${R}${STATE}${NC}  →  journalctl -u ${svc} -n 20"
    fi
done

# ── 5. micro-ROS Agent (proceso) ──────────────────────────
echo -e "\n${B}${BOLD}[ micro-ROS Agent ]${NC}"
if pgrep -f micro_ros_agent &>/dev/null; then
    AGENT_PID=$(pgrep -f micro_ros_agent | head -1)
    echo -e "  Proceso: ${G}CORRIENDO${NC} (PID $AGENT_PID)"
else
    echo -e "  Proceso: ${R}NO ENCONTRADO${NC}"
fi

# ── 6. Firewall ───────────────────────────────────────────
echo -e "\n${B}${BOLD}[ Firewall ]${NC}"
if sudo iptables -L FORWARD -n 2>/dev/null | grep -q "DROP"; then
    echo -e "  Estado: ${G}PROTEGIDO${NC} — Aislamiento ON (FORWARD=DROP)"
elif ! command -v iptables &>/dev/null; then
    echo -e "  Estado: ${Y}iptables no instalado${NC}"
else
    echo -e "  Estado: ${R}ALERTA${NC} — Abierto o sin aplicar"
fi

# ── 7. Logs rápidos ───────────────────────────────────────
echo -e "\n${B}${BOLD}[ Logs Rápidos ]${NC}"
for svc in smart-agent smart-bridge; do
    echo -e "  ${Y}— $svc (último mensaje):${NC}"
    LAST=$(sudo journalctl -u "$svc" -n 1 --no-pager -o short 2>/dev/null \
           | grep -v "^--" | tail -1 || true)
    if [[ -n "$LAST" ]]; then
        # Truncar si es muy largo
        echo "    ${LAST:0:120}"
    else
        echo -e "    ${R}Sin registros (¿servicio nunca iniciado?)${NC}"
    fi
done

divider
