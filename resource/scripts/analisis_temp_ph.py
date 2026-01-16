"""
Script para analizar datos de pH desde un archivo JSON y crear una serie de tiempo.
Autor: Sistema de Monitoreo de Sensores
Fecha: 2026-01-15
"""

import json
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime
import numpy as np
from pathlib import Path

# Configuración de estilo para gráficos
sns.set_style("whitegrid")
plt.rcParams['figure.figsize'] = (14, 8)
plt.rcParams['font.size'] = 10


def cargar_datos_json(ruta_archivo):
    """
    Carga los datos desde un archivo JSON.
    
    Args:
        ruta_archivo (str): Ruta al archivo JSON
        
    Returns:
        list: Lista de diccionarios con los datos
    """
    try:
        with open(ruta_archivo, 'r', encoding='utf-8') as f:
            datos = json.load(f)
        print(f"✓ Datos cargados exitosamente: {len(datos)} registros")
        return datos
    except FileNotFoundError:
        print(f"✗ Error: No se encontró el archivo {ruta_archivo}")
        return None
    except json.JSONDecodeError as e:
        print(f"✗ Error al decodificar JSON: {e}")
        return None


def extraer_serie_ph(datos):
    """
    Extrae la serie de tiempo de pH desde los datos JSON.
    
    Args:
        datos (list): Lista de diccionarios con los datos
        
    Returns:
        pd.DataFrame: DataFrame con timestamp y pH
    """
    registros = []
    
    for registro in datos:
        try:
            # Extraer timestamp
            timestamp = registro.get('timestamp')
            
            # Extraer valor de pH
            ph_info = registro.get('ph', {})
            ph_valor = ph_info.get('valor')
            
            if timestamp and ph_valor is not None:
                registros.append({
                    'timestamp': timestamp,
                    'ph': ph_valor
                })
        except Exception as e:
            print(f"⚠ Advertencia: Error procesando registro: {e}")
            continue
    
    # Crear DataFrame
    df = pd.DataFrame(registros)
    
    # Convertir timestamp a datetime
    df['timestamp'] = pd.to_datetime(df['timestamp'])
    
    # Ordenar por timestamp
    df = df.sort_values('timestamp').reset_index(drop=True)
    
    print(f"✓ Serie de tiempo creada: {len(df)} puntos de datos")
    
    return df


def estadisticas_ph(df):
    """
    Calcula estadísticas descriptivas del pH.
    
    Args:
        df (pd.DataFrame): DataFrame con los datos de pH
    """
    print("\n" + "="*60)
    print("ESTADÍSTICAS DESCRIPTIVAS DE pH")
    print("="*60)
    print(f"Cantidad de mediciones: {len(df)}")
    print(f"Media: {df['ph'].mean():.2f}")
    print(f"Mediana: {df['ph'].median():.2f}")
    print(f"Desviación estándar: {df['ph'].std():.2f}")
    print(f"Mínimo: {df['ph'].min():.2f}")
    print(f"Máximo: {df['ph'].max():.2f}")
    print(f"Rango: {df['ph'].max() - df['ph'].min():.2f}")
    
    # Percentiles
    print(f"\nPercentiles:")
    print(f"  25%: {df['ph'].quantile(0.25):.2f}")
    print(f"  50%: {df['ph'].quantile(0.50):.2f}")
    print(f"  75%: {df['ph'].quantile(0.75):.2f}")
    
    # Información temporal
    print(f"\nRango temporal:")
    print(f"  Inicio: {df['timestamp'].min()}")
    print(f"  Fin: {df['timestamp'].max()}")
    print(f"  Duración: {df['timestamp'].max() - df['timestamp'].min()}")
    print("="*60 + "\n")


def graficar_serie_tiempo(df, guardar=True):
    """
    Genera visualizaciones de la serie de tiempo de pH en archivos separados.
    
    Args:
        df (pd.DataFrame): DataFrame con los datos de pH
        guardar (bool): Si True, guarda las gráficas como archivos
    """
    # Crear directorio de salida si no existe
    output_dir = Path(__file__).parent.parent / 'images' / 'serie_tiempo_ph'
    output_dir.mkdir(parents=True, exist_ok=True)
    
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Gráfico 1: Serie de tiempo completa
    fig1, ax1 = plt.subplots(figsize=(14, 6))
    ax1.plot(df['timestamp'], df['ph'], linewidth=1.5, color='#2E86AB', alpha=0.8)
    ax1.axhline(y=7.0, color='green', linestyle='--', linewidth=1, label='pH neutro (7.0)')
    ax1.set_xlabel('Tiempo', fontsize=11)
    ax1.set_ylabel('pH', fontsize=11)
    ax1.set_title('Serie de Tiempo - Valores de pH', fontsize=13, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    plt.tight_layout()
    
    if guardar:
        ruta_salida1 = output_dir / f'ph_serie_tiempo_{timestamp_str}.png'
        plt.savefig(ruta_salida1, dpi=300, bbox_inches='tight')
        print(f"✓ Gráfica 1 guardada en: {ruta_salida1}")
    plt.show()
    plt.close()
    
    # Gráfico 2: Serie de tiempo con media móvil
    window = min(20, len(df) // 10)  # Ventana adaptativa
    if window >= 2:
        df['ph_ma'] = df['ph'].rolling(window=window, center=True).mean()
        
        fig2, ax2 = plt.subplots(figsize=(14, 6))
        ax2.plot(df['timestamp'], df['ph'], linewidth=0.8, color='#2E86AB', alpha=0.4, label='pH medido')
        ax2.plot(df['timestamp'], df['ph_ma'], linewidth=2, color='#A23B72', label=f'Media móvil ({window} puntos)')
        ax2.set_xlabel('Tiempo', fontsize=11)
        ax2.set_ylabel('pH', fontsize=11)
        ax2.set_title('Serie de Tiempo con Media Móvil', fontsize=13, fontweight='bold')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        plt.tight_layout()
        
        if guardar:
            ruta_salida2 = output_dir / f'ph_media_movil_{timestamp_str}.png'
            plt.savefig(ruta_salida2, dpi=300, bbox_inches='tight')
            print(f"✓ Gráfica 2 guardada en: {ruta_salida2}")
        plt.show()
        plt.close()
    
    # Gráfico 3: Distribución del pH
    fig3, ax3 = plt.subplots(figsize=(12, 6))
    ax3.hist(df['ph'], bins=30, color='#18A558', alpha=0.7, edgecolor='black')
    ax3.axvline(df['ph'].mean(), color='red', linestyle='--', linewidth=2, label=f'Media: {df["ph"].mean():.2f}')
    ax3.axvline(df['ph'].median(), color='orange', linestyle='--', linewidth=2, label=f'Mediana: {df["ph"].median():.2f}')
    ax3.set_xlabel('pH', fontsize=11)
    ax3.set_ylabel('Frecuencia', fontsize=11)
    ax3.set_title('Distribución de Valores de pH', fontsize=13, fontweight='bold')
    ax3.legend()
    ax3.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    
    if guardar:
        ruta_salida3 = output_dir / f'ph_distribucion_{timestamp_str}.png'
        plt.savefig(ruta_salida3, dpi=300, bbox_inches='tight')
        print(f"✓ Gráfica 3 guardada en: {ruta_salida3}")
    plt.show()
    plt.close()


def detectar_anomalias(df, umbral_std=2):
    """
    Detecta valores anómalos en la serie de pH usando desviación estándar.
    
    Args:
        df (pd.DataFrame): DataFrame con los datos de pH
        umbral_std (float): Número de desviaciones estándar para considerar anomalía
    """
    media = df['ph'].mean()
    std = df['ph'].std()
    
    limite_inferior = media - umbral_std * std
    limite_superior = media + umbral_std * std
    
    anomalias = df[(df['ph'] < limite_inferior) | (df['ph'] > limite_superior)]
    
    print(f"\n{'='*60}")
    print(f"DETECCIÓN DE ANOMALÍAS (>{umbral_std}σ)")
    print(f"{'='*60}")
    print(f"Límite inferior: {limite_inferior:.2f}")
    print(f"Límite superior: {limite_superior:.2f}")
    print(f"Anomalías detectadas: {len(anomalias)} ({len(anomalias)/len(df)*100:.2f}%)")
    
    if len(anomalias) > 0:
        print("\nPrimeros valores anómalos:")
        print(anomalias.head(10).to_string(index=False))
    print(f"{'='*60}\n")
    
    return anomalias


def main():
    """
    Función principal del script.
    """
    print("\n" + "="*60)
    print("ANÁLISIS DE SERIE DE TIEMPO - pH")
    print("="*60 + "\n")
    
    # Ruta al archivo JSON (ajustar según sea necesario)
    ruta_json = Path(__file__).parent.parent / 'data' / 'Datos_ESP.datos_sensoresV3.json'
    
    # Permitir especificar archivo por argumento
    import sys
    if len(sys.argv) > 1:
        ruta_json = Path(sys.argv[1])
    
    print(f"Archivo de entrada: {ruta_json}\n")
    
    # Cargar datos
    datos = cargar_datos_json(ruta_json)
    if datos is None:
        return
    
    # Extraer serie de pH
    df = extraer_serie_ph(datos)
    if df.empty:
        print("✗ Error: No se pudieron extraer datos de pH")
        return
    
    # Calcular estadísticas
    estadisticas_ph(df)
    
    # Detectar anomalías
    anomalias = detectar_anomalias(df, umbral_std=2)
    
    # Generar gráficas
    graficar_serie_tiempo(df, guardar=True)
    
    print("\n✓ Análisis completado exitosamente")


if __name__ == "__main__":
    main()
