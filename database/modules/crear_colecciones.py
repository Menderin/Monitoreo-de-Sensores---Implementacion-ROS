#!/usr/bin/env python3
"""
Script de migraciones para crear las colecciones de MongoDB.
Ejecutar una sola vez para configurar la base de datos.

Uso:
    python migrations.py
"""

import os
from pathlib import Path
from datetime import datetime, timezone

from dotenv import load_dotenv, set_key
from pymongo import MongoClient, ASCENDING, DESCENDING
from pymongo.errors import CollectionInvalid

# ═══════════════════════════════════════════════════════════════════
# CONFIGURACIÓN - Nombres de las colecciones
# ═══════════════════════════════════════════════════════════════════
COLECCION_DATOS = "sensors_data"       # Time Series Collection
COLECCION_DISPOSITIVOS = "devices_data"  # Metadatos

ENV_PATH = Path(__file__).parent.parent / '.env'

# ═══════════════════════════════════════════════════════════════════


def cargar_env():
    """Carga las variables de entorno."""
    load_dotenv(ENV_PATH)


def conectar_mongodb():
    """Conecta a MongoDB Atlas y retorna el cliente."""
    uri = os.getenv('MONGO_URI')
    db_name = os.getenv('MONGO_DB', 'Datos_ESP')
    
    if not uri:
        raise ValueError("MONGO_URI no configurado en .env")
    
    client = MongoClient(uri, serverSelectionTimeoutMS=5000)
    client.admin.command('ping')
    print(f"[OK] Conectado a MongoDB Atlas")
    
    return client, client[db_name]


def crear_coleccion_datos_sensores(db):
    """
    Crea la colección Time Series para datos de sensores.
    """
    nombre = COLECCION_DATOS
    
    if nombre in db.list_collection_names():
        print(f"[AVISO] Colección '{nombre}' ya existe, omitiendo creación")
        return False
    
    try:
        db.create_collection(
            nombre,
            timeseries={
                "timeField": "timestamp",
                "metaField": "dispositivo_id", 
                "granularity": "minutes"
            }
        )
        print(f"[OK] Colección '{nombre}' creada (Time Series)")
        
        # Crear índice
        db[nombre].create_index([
            ("dispositivo_id", ASCENDING),
            ("timestamp", DESCENDING)
        ])
        print(f"   [OK] Índice dispositivo_id + timestamp creado")
        
        return True
        
    except CollectionInvalid as e:
        print(f"[ERROR] Error creando colección: {e}")
        return False


def crear_coleccion_info_dispositivos(db):
    """
    Crea la colección para metadatos de dispositivos.
    """
    nombre = COLECCION_DISPOSITIVOS
    
    if nombre in db.list_collection_names():
        print(f"[AVISO] Colección '{nombre}' ya existe, omitiendo creación")
        return False
    
    db.create_collection(nombre)
    print(f"[OK] Colección '{nombre}' creada")
    
    # Índices
    db[nombre].create_index("estado")
    print(f"    Índice 'estado' creado")
    
    return True


def insertar_dispositivo_ejemplo(db):
    """
    Inserta un dispositivo de ejemplo.
    """
    dispositivo = {
        "_id": "esp32_default",
        "nombre": "Dispositivo de Prueba",
        "estado": "pendiente",
        "auto_registrado": False,
        "ubicacion": None,
        "configuracion": {
            "firmware_version": "1.0.0",
            "intervalo_lectura_seg": 60,
            "sensores_habilitados": ["ph", "temperatura"]
        },
        "calibracion": {
            "ph_offset": 0.0,
            "temp_offset": 0.0
        },
        "unidades": {
            "temperatura": "°C",
            "ph": "pH"
        },
        "conexion": {
            "primera": datetime.now(timezone.utc),
            "ultima": datetime.now(timezone.utc),
            "total_lecturas": 0
        }
    }
    
    try:
        db[COLECCION_DISPOSITIVOS].insert_one(dispositivo)
        print(f"[OK] Dispositivo de ejemplo insertado: {dispositivo['_id']}")
        return True
    except Exception as e:
        print(f"[AVISO] Dispositivo ya existe o error: {e}")
        return False

def actualizar_env_colecciones():
    """
    Actualiza el archivo .env con los nombres de las colecciones.
    """
    print("\nActualizando .env con nombres de colecciones...")
    
    try:
        set_key(ENV_PATH, "MONGO_COLLECTION", COLECCION_DATOS)
        print(f"   [OK] MONGO_COLLECTION = {COLECCION_DATOS}")
        
        set_key(ENV_PATH, "MONGO_COLLECTION_DISPOSITIVOS", COLECCION_DISPOSITIVOS)
        print(f"   [OK] MONGO_COLLECTION_DISPOSITIVOS = {COLECCION_DISPOSITIVOS}")
        
        return True
    except Exception as e:
        print(f"   [ERROR] Error actualizando .env: {e}")
        return False


def mostrar_resumen(db):
    """Muestra resumen de colecciones."""
    print("\nCOLECCIONES EN LA BASE DE DATOS:")
    for col in db.list_collection_names():
        if not col.startswith('system.'):
            count = db[col].count_documents({})
            print(f"   - {col}: {count} documentos")


def ejecutar_migraciones():
    """Ejecuta la creación de colecciones."""
    print("\n" + "=" * 60)
    print("CREANDO COLECCIONES")
    print("=" * 60)
    
    cargar_env()
    client, db = conectar_mongodb()
    print(f"Base de datos: {db.name}")
    print("-" * 60)
    
    crear_coleccion_datos_sensores(db)
    crear_coleccion_info_dispositivos(db)
    insertar_dispositivo_ejemplo(db)
    
    mostrar_resumen(db)
    client.close()


def menu_principal():
    """Menú interactivo principal."""
    print("\n" + "═" * 60)
    print("   CONFIGURACIÓN DE BASE DE DATOS IoT")
    print("═" * 60)
    print(f"\n   Colección de datos:       {COLECCION_DATOS}")
    print(f"   Colección de dispositivos: {COLECCION_DISPOSITIVOS}")
    print(f"   Archivo .env:              {ENV_PATH}")
    print("\n" + "-" * 60)
    print("\n   OPCIONES:\n")
    print("   [1] Crear colecciones en MongoDB")
    print("   [2] Crear colecciones + actualizar .env")
    print("   [3] Solo actualizar .env (sin crear colecciones)")
    print("   [0] Salir")
    print("\n" + "-" * 60)
    
    opcion = input("\n   Selecciona una opción: ").strip()
    
    if opcion == "1":
        ejecutar_migraciones()
        
    elif opcion == "2":
        ejecutar_migraciones()
        actualizar_env_colecciones()
        print("\n[OK] Todo completado")

        
    elif opcion == "0":
        print("\nSaliendo...")
        
    else:
        print("\n[ERROR] Opción no válida")


if __name__ == "__main__":
    try:
        menu_principal()
    except KeyboardInterrupt:
        print("\n\nCancelado por el usuario")
    except Exception as e:
        print(f"\n[ERROR] {e}")
