"""
Script para analizar datos de sensores (pH y temperatura) desde un archivo JSON y crear series de tiempo.
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

# Configuración de variables
VARIABLES_CONFIG = {
    'ph': {
        'nombre': 'pH',
        'unidad': 'pH',
        'color_principal': '#2E86AB',
        'color_secundario': '#A23B72',
        'color_histograma': '#18A558',
        'linea_referencia': {'valor': 7.0, 'label': 'pH neutro (7.0)', 'color': 'green'}
    },
    'temperatura': {
        'nombre': 'Temperatura',
        'unidad': '°C',
        'color_principal': '#E63946',
        'color_secundario': '#F77F00',
        'color_histograma': '#06AED5',
        'linea_referencia': None
    }
}


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


def extraer_serie_variable(datos, variable):
    """
    Extrae la serie de tiempo de una variable desde los datos JSON.
    
    Args:
        datos (list): Lista de diccionarios con los datos
        variable (str): Nombre de la variable a extraer ('ph' o 'temperatura')
        
    Returns:
        pd.DataFrame: DataFrame con timestamp y variable
    """
    registros = []
    
    for registro in datos:
        try:
            timestamp = registro.get('timestamp')
            var_info = registro.get(variable, {})
            var_valor = var_info.get('valor')
            
            if timestamp and var_valor is not None:
                registros.append({
                    'timestamp': timestamp,
                    variable: var_valor
                })
        except Exception as e:
            print(f"⚠ Advertencia: Error procesando registro: {e}")
            continue
    
    df = pd.DataFrame(registros)
    df['timestamp'] = pd.to_datetime(df['timestamp'])
    df = df.sort_values('timestamp').reset_index(drop=True)
    
    print(f"✓ Serie de tiempo de {variable} creada: {len(df)} puntos de datos")
    
    return df


def estadisticas_variable(df, variable):
    """
    Calcula estadísticas descriptivas de una variable.
    
    Args:
        df (pd.DataFrame): DataFrame con los datos
        variable (str): Nombre de la variable
    """
    config = VARIABLES_CONFIG.get(variable, {})
    nombre = config.get('nombre', variable)
    unidad = config.get('unidad', '')
    
    print("\n" + "="*60)
    print(f"ESTADÍSTICAS DESCRIPTIVAS DE {nombre.upper()}")
    print("="*60)
    print(f"Cantidad de mediciones: {len(df)}")
    print(f"Media: {df[variable].mean():.2f} {unidad}")
    print(f"Mediana: {df[variable].median():.2f} {unidad}")
    print(f"Desviación estándar: {df[variable].std():.2f} {unidad}")
    print(f"Mínimo: {df[variable].min():.2f} {unidad}")
    print(f"Máximo: {df[variable].max():.2f} {unidad}")
    print(f"Rango: {df[variable].max() - df[variable].min():.2f} {unidad}")
    
    print(f"\nPercentiles:")
    print(f"  25%: {df[variable].quantile(0.25):.2f} {unidad}")
    print(f"  50%: {df[variable].quantile(0.50):.2f} {unidad}")
    print(f"  75%: {df[variable].quantile(0.75):.2f} {unidad}")
    
    print(f"\nRango temporal:")
    print(f"  Inicio: {df['timestamp'].min()}")
    print(f"  Fin: {df['timestamp'].max()}")
    print(f"  Duración: {df['timestamp'].max() - df['timestamp'].min()}")
    print("="*60 + "\n")


def graficar_serie_temporal(df, variable, output_dir, timestamp_str):
    """
    Genera gráfico de serie temporal para una variable.
    
    Args:
        df (pd.DataFrame): DataFrame con los datos
        variable (str): Nombre de la variable
        output_dir (Path): Directorio de salida
        timestamp_str (str): String de timestamp para el nombre del archivo
    """
    config = VARIABLES_CONFIG.get(variable, {})
    nombre = config.get('nombre', variable)
    unidad = config.get('unidad', '')
    color = config.get('color_principal', '#2E86AB')
    ref = config.get('linea_referencia')
    
    fig, ax = plt.subplots(figsize=(14, 6))
    ax.plot(df['timestamp'], df[variable], linewidth=1.5, color=color, alpha=0.8)
    
    if ref:
        ax.axhline(y=ref['valor'], color=ref['color'], linestyle='--', 
                  linewidth=1, label=ref['label'])
    
    ax.set_xlabel('Tiempo', fontsize=11)
    ax.set_ylabel(f'{nombre} ({unidad})', fontsize=11)
    ax.set_title(f'Serie de Tiempo - Valores de {nombre}', fontsize=13, fontweight='bold')
    if ref:
        ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    
    ruta_salida = output_dir / f'{variable}_serie_tiempo_{timestamp_str}.png'
    plt.savefig(ruta_salida, dpi=300, bbox_inches='tight')
    print(f"✓ Serie temporal de {nombre} guardada en: {ruta_salida}")
    plt.close()


def graficar_media_movil(df, variable, output_dir, timestamp_str, window=None):
    """
    Genera gráfico de serie temporal con media móvil.
    
    Args:
        df (pd.DataFrame): DataFrame con los datos
        variable (str): Nombre de la variable
        output_dir (Path): Directorio de salida
        timestamp_str (str): String de timestamp para el nombre del archivo
        window (int): Tamaño de ventana para media móvil
    """
    if window is None:
        window = min(20, len(df) // 10)
    
    if window < 2:
        return
    
    config = VARIABLES_CONFIG.get(variable, {})
    nombre = config.get('nombre', variable)
    unidad = config.get('unidad', '')
    color_principal = config.get('color_principal', '#2E86AB')
    color_secundario = config.get('color_secundario', '#A23B72')
    
    df[f'{variable}_ma'] = df[variable].rolling(window=window, center=True).mean()
    
    fig, ax = plt.subplots(figsize=(14, 6))
    ax.plot(df['timestamp'], df[variable], linewidth=0.8, color=color_principal, 
           alpha=0.4, label=f'{nombre} medido')
    ax.plot(df['timestamp'], df[f'{variable}_ma'], linewidth=2, color=color_secundario, 
           label=f'Media móvil ({window} puntos)')
    ax.set_xlabel('Tiempo', fontsize=11)
    ax.set_ylabel(f'{nombre} ({unidad})', fontsize=11)
    ax.set_title(f'Serie de Tiempo con Media Móvil - {nombre}', fontsize=13, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    
    ruta_salida = output_dir / f'{variable}_media_movil_{timestamp_str}.png'
    plt.savefig(ruta_salida, dpi=300, bbox_inches='tight')
    print(f"✓ Media móvil de {nombre} guardada en: {ruta_salida}")
    plt.close()


def graficar_histograma(df, variable, output_dir, timestamp_str):
    """
    Genera histograma de distribución para una variable.
    
    Args:
        df (pd.DataFrame): DataFrame con los datos
        variable (str): Nombre de la variable
        output_dir (Path): Directorio de salida
        timestamp_str (str): String de timestamp para el nombre del archivo
    """
    config = VARIABLES_CONFIG.get(variable, {})
    nombre = config.get('nombre', variable)
    unidad = config.get('unidad', '')
    color = config.get('color_histograma', '#18A558')
    
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.hist(df[variable], bins=30, color=color, alpha=0.7, edgecolor='black')
    ax.axvline(df[variable].mean(), color='red', linestyle='--', linewidth=2, 
              label=f'Media: {df[variable].mean():.2f} {unidad}')
    ax.axvline(df[variable].median(), color='orange', linestyle='--', linewidth=2, 
              label=f'Mediana: {df[variable].median():.2f} {unidad}')
    ax.set_xlabel(f'{nombre} ({unidad})', fontsize=11)
    ax.set_ylabel('Frecuencia', fontsize=11)
    ax.set_title(f'Distribución de Valores de {nombre}', fontsize=13, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    
    ruta_salida = output_dir / f'{variable}_distribucion_{timestamp_str}.png'
    plt.savefig(ruta_salida, dpi=300, bbox_inches='tight')
    print(f"✓ Histograma de {nombre} guardado en: {ruta_salida}")
    plt.close()


def generar_graficas_variable(df, variable):
    """
    Genera todas las gráficas para una variable.
    
    Args:
        df (pd.DataFrame): DataFrame con los datos
        variable (str): Nombre de la variable
    """
    output_dir = Path(__file__).parent.parent / 'images' / f'serie_tiempo_{variable}'
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    graficar_serie_temporal(df, variable, output_dir, timestamp_str)
    graficar_media_movil(df, variable, output_dir, timestamp_str)
    graficar_histograma(df, variable, output_dir, timestamp_str)


def detectar_anomalias(df, variable, umbral_std=2):
    """
    Detecta valores anómalos en una serie usando desviación estándar.
    
    Args:
        df (pd.DataFrame): DataFrame con los datos
        variable (str): Nombre de la variable
        umbral_std (float): Número de desviaciones estándar para considerar anomalía
    """
    config = VARIABLES_CONFIG.get(variable, {})
    nombre = config.get('nombre', variable)
    unidad = config.get('unidad', '')
    
    media = df[variable].mean()
    std = df[variable].std()
    
    limite_inferior = media - umbral_std * std
    limite_superior = media + umbral_std * std
    
    anomalias = df[(df[variable] < limite_inferior) | (df[variable] > limite_superior)]
    
    print(f"\n{'='*60}")
    print(f"DETECCIÓN DE ANOMALÍAS EN {nombre.upper()} (>{umbral_std}σ)")
    print(f"{'='*60}")
    print(f"Límite inferior: {limite_inferior:.2f} {unidad}")
    print(f"Límite superior: {limite_superior:.2f} {unidad}")
    print(f"Anomalías detectadas: {len(anomalias)} ({len(anomalias)/len(df)*100:.2f}%)")
    
    if len(anomalias) > 0:
        print("\nPrimeros valores anómalos:")
        print(anomalias.head(10).to_string(index=False))
    print(f"{'='*60}\n")
    
    return anomalias


def analizar_variable(datos, variable):
    """
    Realiza el análisis completo de una variable.
    
    Args:
        datos (list): Lista de diccionarios con los datos
        variable (str): Nombre de la variable a analizar
    """
    config = VARIABLES_CONFIG.get(variable, {})
    nombre = config.get('nombre', variable)
    
    print("\n" + "="*60)
    print(f"ANÁLISIS DE {nombre.upper()}")
    print("="*60 + "\n")
    
    # Extraer serie
    df = extraer_serie_variable(datos, variable)
    if df.empty:
        print(f"✗ Error: No se pudieron extraer datos de {nombre}")
        return
    
    # Estadísticas
    estadisticas_variable(df, variable)
    
    # Detectar anomalías
    detectar_anomalias(df, variable, umbral_std=2)
    
    # Generar gráficas
    generar_graficas_variable(df, variable)


def main():
    """
    Función principal del script.
    """
    print("\n" + "="*60)
    print("ANÁLISIS DE SERIES DE TIEMPO - SENSORES")
    print("="*60 + "\n")
    
    # Ruta al archivo JSON
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
    
    # Analizar pH
    analizar_variable(datos, 'ph')
    
    # Analizar temperatura
    analizar_variable(datos, 'temperatura')
    
    print("\n✓ Análisis completado exitosamente")


if __name__ == "__main__":
    main()
