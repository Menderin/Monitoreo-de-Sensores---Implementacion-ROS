#!/usr/bin/env python3
"""
Script para generar wifi_config.h desde .env
Se ejecuta automáticamente antes de compilar
"""
import os
import sys

# Obtener directorio del script de forma robusta (funciona con CMake)
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))

# Ruta al archivo .env
ENV_FILE = os.path.join(SCRIPT_DIR, '.env')
OUTPUT_FILE = os.path.join(SCRIPT_DIR, 'wifi_config.h')

def parse_env_file(filepath):
    """Lee el archivo .env y retorna un diccionario"""
    config = {}
    try:
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                # Ignorar comentarios y líneas vacías
                if line and not line.startswith('#'):
                    if '=' in line:
                        key, value = line.split('=', 1)
                        # Limpiar: quitar \r (Windows), espacios, y comillas envolventes
                        value = value.strip().strip('\r').strip('"').strip("'")
                        config[key.strip()] = value
        return config
    except FileNotFoundError:
        print(f"❌ ERROR: No se encontró el archivo .env en: {filepath}")
        print(f"💡 Copia .env.example a .env y completa las credenciales")
        sys.exit(1)

def generate_header(config):
    """Genera el archivo wifi_config.h"""
    
    # Validar que existan las claves necesarias
    required = ['WIFI_SSID', 'WIFI_PASSWORD', 'AGENT_IP', 'AGENT_PORT']
    missing = [key for key in required if not config.get(key)]
    
    if missing:
        print(f"❌ ERROR: Faltan las siguientes variables en .env: {', '.join(missing)}")
        sys.exit(1)
    
    # Validar que no estén vacías
    empty = [key for key in required if not config.get(key).strip()]
    if empty:
        print(f"⚠️  ADVERTENCIA: Las siguientes variables están vacías: {', '.join(empty)}")
        print(f"💡 Completa el archivo .env antes de compilar")
        sys.exit(1)
    
    header_content = f"""/**
 * @file wifi_config.h
 * @brief Configuración WiFi y micro-ROS Agent
 * 
 * GENERADO AUTOMÁTICAMENTE desde .env
 * NO EDITAR MANUALMENTE - Los cambios se perderán
 * 
 * Para modificar: edita el archivo .env en la raíz del proyecto
 */

#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

// Configuración WiFi
// Nota: el número de reintentos y los tiempos de espera se definen
// en config.h (WIFI_MAX_RECONNECT_TIME_MS, WIFI_RETRY_DELAY_MS, etc.)
#define WIFI_SSID           "{config['WIFI_SSID']}"
#define WIFI_PASSWORD       "{config['WIFI_PASSWORD']}"

// Configuración micro-ROS Agent (UDP)
#define AGENT_IP            "{config['AGENT_IP']}"
#define AGENT_PORT          {config['AGENT_PORT']}

#endif // WIFI_CONFIG_H
"""
    
    with open(OUTPUT_FILE, 'w') as f:
        f.write(header_content)
    
    print(f"✅ Generado: {OUTPUT_FILE}")
    print(f"   SSID: {config['WIFI_SSID']}")
    print(f"   Agent: {config['AGENT_IP']}:{config['AGENT_PORT']}")

if __name__ == '__main__':
    config = parse_env_file(ENV_FILE)
    generate_header(config)
