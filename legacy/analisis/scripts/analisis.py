import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os

# Crear carpeta para guardar gráficos si no existe
os.makedirs('images/histogramas y ST', exist_ok=True)

df = pd.read_excel('Sensores TACHI.xlsx')

# Mantener las primeras 7 columnas (incluyendo luz)
df = df.iloc[:, :7]

# Eliminar filas con valores anormales
print("\n" + "="*60)
print("LIMPIEZA DE DATOS ANORMALES")
print("="*60)

# Valores -1 (fallo del sensor)
print("\nValores -1 por columna:")
print((df == -1).sum())
filas_antes = len(df)
df = df[(df != -1).all(axis=1)]

# Eliminar valores fuera de rangos físicamente posibles
# Temperatura: rango razonable para agua [-10°C a 40°C]
df = df[(df['temperatura'] >= -10) & (df['temperatura'] <= 40)]
# pH: rango válido [0 a 14]
df = df[(df['ph'] >= 0) & (df['ph'] <= 14)]
# Oxígeno: valores positivos razonables [0 a 20 mg/L típico]
df = df[(df['oxigeno'] >= 0) & (df['oxigeno'] <= 20)]
# Turbidez y luz: solo positivos
df = df[(df['turbidez'] >= 0) & (df['luz'] >= 0)]

filas_eliminadas = filas_antes - len(df)
print(f"\nFilas eliminadas por valores anormales: {filas_eliminadas}")
print(f"Filas restantes: {len(df)}")

# Mostrar valores nulos antes de eliminar filas
print("\nValores nulos por columna (antes):")
print(df.isnull().sum())
print(f"Total de filas antes: {len(df)}")

# Eliminar todas las filas con valores nulos
columnas_numericas = ['temperatura', 'ph', 'turbidez', 'oxigeno', 'luz']
filas_antes_nulos = len(df)
df = df.dropna(subset=columnas_numericas)
filas_eliminadas_nulos = filas_antes_nulos - len(df)
print(f"Filas eliminadas por valores nulos: {filas_eliminadas_nulos}")

# Verificar que no quedan valores nulos
print("\nValores nulos por columna (después):")
print(df.isnull().sum())
print(f"Total de filas después: {len(df)}")

print(f"\nPrimeras filas:")
print(df.head())
print(f"\nInformación del dataset:")
print(df.info())

# Verificar valores anormales - Estadísticas descriptivas
print("\n" + "="*60)
print("ESTADÍSTICAS DESCRIPTIVAS (DATOS LIMPIOS)")
print("="*60)
print(df[columnas_numericas].describe())

# Tratamiento especial para LUZ: Eliminar outliers extremos y aplicar transformación logarítmica
print("\n" + "="*60)
print("TRATAMIENTO DE LUZ - OUTLIERS Y TRANSFORMACIÓN")
print("="*60)

# Eliminar valores extremos de luz (>percentil 99)
percentil_99_luz = df['luz'].quantile(0.99)
filas_antes_luz = len(df)
df = df[df['luz'] <= percentil_99_luz]
filas_eliminadas_luz = filas_antes_luz - len(df)
print(f"Percentil 99 de luz: {percentil_99_luz:.2f} Lux")
print(f"Filas eliminadas por outliers extremos de luz: {filas_eliminadas_luz}")
print(f"Filas restantes: {len(df)}")

# Aplicar transformación logarítmica a luz (log(x+1) para manejar valores muy cercanos a 0)
import numpy as np
df['luz_original'] = df['luz'].copy()  # Guardar valores originales
df['luz'] = np.log1p(df['luz'])  # log1p = log(1 + x)
print(f"\nTransformación logarítmica aplicada a luz: log(luz + 1)")
print(f"Nuevo rango de luz transformada: [{df['luz'].min():.2f}, {df['luz'].max():.2f}]")

# Tratamiento especial para TURBIDEZ: Eliminar outliers bajos y aplicar transformación logarítmica
print("\n" + "="*60)
print("TRATAMIENTO DE TURBIDEZ - OUTLIERS Y TRANSFORMACIÓN")
print("="*60)

# Eliminar valores extremadamente bajos de turbidez (<percentil 5)
percentil_5_turbidez = df['turbidez'].quantile(0.05)
filas_antes_turbidez = len(df)
df = df[df['turbidez'] >= percentil_5_turbidez]
filas_eliminadas_turbidez = filas_antes_turbidez - len(df)
print(f"Percentil 5 de turbidez: {percentil_5_turbidez:.2f} NTU")
print(f"Filas eliminadas por outliers extremadamente bajos de turbidez: {filas_eliminadas_turbidez}")
print(f"Filas restantes: {len(df)}")

# Aplicar transformación logarítmica a turbidez
df['turbidez_original'] = df['turbidez'].copy()  # Guardar valores originales
df['turbidez'] = np.log1p(df['turbidez'])  # log1p = log(1 + x)
print(f"\nTransformación logarítmica aplicada a turbidez: log(turbidez + 1)")
print(f"Nuevo rango de turbidez transformada: [{df['turbidez'].min():.2f}, {df['turbidez'].max():.2f}]")

# Estandarización (Z-score)
print("\n" + "="*60)
print("ESTANDARIZACIÓN (Z-SCORE)")
print("="*60)
from sklearn.preprocessing import StandardScaler

scaler = StandardScaler()
df_estandarizado = df.copy()
df_estandarizado[columnas_numericas] = scaler.fit_transform(df[columnas_numericas])

print("Datos estandarizados - Media y Desviación Estándar:")
print(f"Media (debe ser ~0):")
print(df_estandarizado[columnas_numericas].mean())
print(f"\nDesviación estándar (debe ser ~1):")
print(df_estandarizado[columnas_numericas].std())

print("\n" + "="*60)
print("ESTADÍSTICAS DESCRIPTIVAS (DATOS ESTANDARIZADOS)")
print("="*60)
print(df_estandarizado[columnas_numericas].describe())

# Matriz de covarianza (datos estandarizados)
print("\n" + "="*60)
print("MATRIZ DE COVARIANZA (DATOS ESTANDARIZADOS)")
print("="*60)
cov_matrix_std = df_estandarizado[columnas_numericas].cov()
print(cov_matrix_std)
print("\nNOTA: En datos estandarizados, covarianza = correlación")

# Crear gráfico de la matriz de covarianza estandarizada
plt.figure(figsize=(10, 8))
sns.heatmap(cov_matrix_std, annot=True, fmt='.3f', cmap='coolwarm', 
            center=0, square=True, linewidths=1, cbar_kws={"shrink": 0.8},
            vmin=-1, vmax=1)
plt.title('Matriz de Covarianza (Datos Estandarizados) - Sensores TACHI', 
          fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig('images/matriz_covarianza_estandarizada.png', dpi=300, bbox_inches='tight')
print("\nGráfico guardado como 'images/matriz_covarianza_estandarizada.png'")

print("\n" + "="*60)
print("INTERPRETACIÓN")
print("="*60)
print("Variables con mayor relación (covarianza/correlación):")
# Obtener covarianzas sin la diagonal
cov_pairs = cov_matrix_std.unstack()
cov_pairs_no_diag = cov_pairs[cov_pairs.index.get_level_values(0) != cov_pairs.index.get_level_values(1)]
cov_pairs_sorted = cov_pairs_no_diag.abs().sort_values(ascending=False)
for i, (pair, value) in enumerate(cov_pairs_sorted.head(5).items(), 1):
    print(f"{i}. {pair[0]} ↔ {pair[1]}: {value:.3f}")

print("\n" + "="*60)
print("INTERPRETACIÓN")
print("="*60)
print("Varianzas (diagonal de la matriz):")
for col in columnas_numericas:
    print(f"  {col}: {cov_matrix_std.loc[col, col]:.2f}")

print("\nCovarianzas más altas (en valor absoluto):")
# Obtener covarianzas sin la diagonal
cov_pairs = cov_matrix_std.unstack()
cov_pairs_no_diag = cov_pairs[cov_pairs.index.get_level_values(0) != cov_pairs.index.get_level_values(1)]
cov_pairs_sorted = cov_pairs_no_diag.abs().sort_values(ascending=False)
print(cov_pairs_sorted.head(10))

# ============================================================
# FUNCIÓN PARA CREAR SERIES TEMPORALES E HISTOGRAMAS
# ============================================================
from matplotlib.dates import DateFormatter, DayLocator

def crear_analisis_temporal_histograma(df, df_sorted, columna, nombre_variable, color, unidad, referencia=None):
    """
    Crea gráficos de serie temporal e histograma para una variable.
    
    Parámetros:
    - df: DataFrame con los datos
    - df_sorted: DataFrame ordenado por tiempo
    - columna: nombre de la columna a analizar
    - nombre_variable: nombre descriptivo de la variable
    - color: color para los gráficos
    - unidad: unidad de medida
    - referencia: valor de referencia opcional (ej: pH neutro = 7)
    """
    print("\n" + "="*60)
    print(f"ANÁLISIS DE {nombre_variable.upper()}")
    print("="*60)
    
    # Crear figura con 2 subplots
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    
    # 1. Serie temporal
    axes[0].plot(df_sorted['tiempo'], df_sorted[columna], 
                 color=color, linewidth=0.8, alpha=0.7)
    axes[0].set_xlabel('Tiempo', fontsize=12)
    axes[0].set_ylabel(f'{nombre_variable} ({unidad})', fontsize=12)
    axes[0].set_title(f'Serie Temporal - {nombre_variable}', fontsize=14, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    
    # Configurar formato de fechas
    axes[0].xaxis.set_major_locator(DayLocator(interval=5))
    axes[0].xaxis.set_major_formatter(DateFormatter('%d-%b'))
    axes[0].xaxis.set_minor_locator(DayLocator(interval=1))
    axes[0].tick_params(axis='x', rotation=45, labelsize=9)
    
    # Estadísticas
    var_mean = df[columna].mean()
    var_std = df[columna].std()
    axes[0].axhline(y=var_mean, color='blue', linestyle='--', linewidth=2, 
                    label=f'Media: {var_mean:.2f} {unidad}')
    axes[0].axhline(y=var_mean + var_std, color='green', linestyle=':', linewidth=1.5,
                    label=f'±1 Std: {var_std:.2f} {unidad}')
    axes[0].axhline(y=var_mean - var_std, color='green', linestyle=':', linewidth=1.5)
    
    # Línea de referencia opcional
    if referencia is not None:
        axes[0].axhline(y=referencia, color='gray', linestyle='-', linewidth=1, alpha=0.5,
                        label=f'Referencia ({referencia})')
    
    axes[0].legend(loc='upper right')
    
    # 2. Histograma
    axes[1].hist(df[columna], bins=50, color=color, alpha=0.7, edgecolor='black')
    axes[1].set_xlabel(f'{nombre_variable} ({unidad})', fontsize=12)
    axes[1].set_ylabel('Frecuencia', fontsize=12)
    axes[1].set_title(f'Distribución de {nombre_variable}', fontsize=14, fontweight='bold')
    axes[1].grid(True, alpha=0.3, axis='y')
    
    # Líneas de media y mediana
    var_median = df[columna].median()
    axes[1].axvline(x=var_mean, color='blue', linestyle='--', linewidth=2,
                    label=f'Media: {var_mean:.2f} {unidad}')
    axes[1].axvline(x=var_median, color='red', linestyle='--', linewidth=2,
                    label=f'Mediana: {var_median:.2f} {unidad}')
    
    if referencia is not None:
        axes[1].axvline(x=referencia, color='gray', linestyle='-', linewidth=1, alpha=0.5,
                        label='Referencia')
    
    axes[1].legend()
    
    # Estadísticas como texto
    stats_text = f'N = {len(df)}\nMin = {df[columna].min():.2f} {unidad}\nMax = {df[columna].max():.2f} {unidad}\nStd = {var_std:.2f} {unidad}'
    axes[1].text(0.98, 0.97, stats_text, transform=axes[1].transAxes, 
                 fontsize=10, verticalalignment='top', horizontalalignment='right',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Guardar
    plt.tight_layout()
    filename = f'images/histogramas y ST/{columna}_analisis.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"\nGráfico guardado como '{filename}'")
    plt.close()
    
    # Imprimir estadísticas
    print(f"\nEstadísticas de {nombre_variable}:")
    print(f"  Media: {var_mean:.2f} {unidad}")
    print(f"  Mediana: {var_median:.2f} {unidad}")
    print(f"  Desv. Std: {var_std:.2f} {unidad}")
    print(f"  Rango: [{df[columna].min():.2f}, {df[columna].max():.2f}] {unidad}")

# ============================================================
# ANÁLISIS DE TODAS LAS VARIABLES
# ============================================================

# Ordenar por tiempo para series temporales
df_sorted = df.sort_values('tiempo')

# Temperatura
crear_analisis_temporal_histograma(df, df_sorted, 'temperatura', 'Temperatura', 'orangered', '°C')

# pH
crear_analisis_temporal_histograma(df, df_sorted, 'ph', 'pH', 'purple', '', referencia=7)

# Turbidez
crear_analisis_temporal_histograma(df, df_sorted, 'turbidez', 'Turbidez', 'saddlebrown', 'NTU')

# Oxígeno
crear_analisis_temporal_histograma(df, df_sorted, 'oxigeno', 'Oxígeno Disuelto', 'deepskyblue', 'mg/L')

# Luz
crear_analisis_temporal_histograma(df, df_sorted, 'luz', 'Intensidad de Luz', 'gold', 'Lux')

# ============================================================
# GRÁFICOS DE CAJAS Y BIGOTES - IMPACTO DE VARIABLES
# ============================================================

def crear_categorias(serie):
    """
    Divide una serie en 3 categorías basadas en percentiles.
    
    Parámetros:
    - serie: Serie de pandas con los datos
    
    Retorna:
    - Serie categórica con etiquetas: 'Bajo', 'Medio', 'Alto'
    """
    q33 = serie.quantile(0.33)
    q66 = serie.quantile(0.67)
    return pd.cut(serie, bins=[serie.min()-0.1, q33, q66, serie.max()+0.1],
                  labels=['Bajo', 'Medio', 'Alto'])


def crear_boxplot_impacto(df_estandarizado, columna_principal, nombre_principal, 
                          color_principal, variables_config, carpeta_salida='images/cajas y bigotes'):
    """
    Crea un gráfico de cajas y bigotes mostrando el impacto de una variable 
    sobre las demás variables del sistema.
    
    Parámetros:
    - df_estandarizado: DataFrame con datos estandarizados (z-score)
    - columna_principal: nombre de la columna principal a analizar
    - nombre_principal: nombre descriptivo de la variable principal
    - color_principal: color para los gráficos (no usado actualmente)
    - variables_config: lista de tuplas (columna, nombre, color) con todas las variables
    - carpeta_salida: carpeta donde guardar los gráficos
    
    Retorna:
    - nombre del archivo guardado
    """
    print(f"\nCreando análisis de impacto para {nombre_principal}...")
    
    # Crear copia para no modificar el original
    df_temp = df_estandarizado.copy()
    
    # Crear categorías para la variable principal
    df_temp['categoria_temp'] = crear_categorias(df_temp[columna_principal])
    
    # Obtener las otras 4 variables
    otras_variables = [v for v in variables_config if v[0] != columna_principal]
    
    # Crear figura con 2x2 subplots
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    axes = axes.flatten()
    
    fig.suptitle(f'Impacto de {nombre_principal} en las demás variables (Datos Estandarizados)\n(Dividido en rangos: Bajo, Medio, Alto)',
                 fontsize=16, fontweight='bold', y=0.995)
    
    for idx, (col, nombre, color) in enumerate(otras_variables):
        ax = axes[idx]
        
        # Preparar datos estandarizados para boxplot
        datos_por_categoria = [df_temp[df_temp['categoria_temp'] == cat][col].dropna() 
                               for cat in ['Bajo', 'Medio', 'Alto']]
        
        # Crear boxplot
        bp = ax.boxplot(datos_por_categoria, labels=['Bajo', 'Medio', 'Alto'],
                        patch_artist=True, showmeans=True, widths=0.6,
                        boxprops=dict(facecolor=color, alpha=0.7),
                        medianprops=dict(color='red', linewidth=2),
                        meanprops=dict(marker='D', markerfacecolor='blue', markersize=8))
        
        ax.set_xlabel(f'{nombre_principal} (z-score)', fontsize=11, fontweight='bold')
        ax.set_ylabel(f'{nombre} (z-score)', fontsize=11)
        ax.set_title(f'{nombre} vs {nombre_principal}', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3, axis='y')
        ax.axhline(y=0, color='gray', linestyle='--', linewidth=1, alpha=0.5, label='Media (0)')
        
        # Agregar conteo de observaciones
        counts = [len(d) for d in datos_por_categoria]
        for i, (count, cat) in enumerate(zip(counts, ['Bajo', 'Medio', 'Alto']), 1):
            ax.text(i, ax.get_ylim()[1]*0.95, f'n={count}', 
                   ha='center', fontsize=9, style='italic')
    
    plt.tight_layout()
    filename = f'{carpeta_salida}/{columna_principal}_impacto.png'
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Gráfico guardado: {filename}")
    plt.close()
    
    return filename


# Crear carpeta para cajas y bigotes
os.makedirs('images/cajas y bigotes', exist_ok=True)

print("\n" + "="*60)
print("GRÁFICOS DE CAJAS Y BIGOTES - ANÁLISIS DE IMPACTO (DATOS ESTANDARIZADOS)")
print("="*60)

# Configuración de variables
variables_config = [
    ('temperatura', 'Temperatura', 'orangered'),
    ('ph', 'pH', 'purple'),
    ('turbidez', 'Turbidez', 'saddlebrown'),
    ('oxigeno', 'Oxígeno Disuelto', 'deepskyblue'),
    ('luz', 'Luz', 'gold')
]

# Crear un gráfico para cada variable mostrando su impacto en las demás
for columna_principal, nombre_principal, color_principal in variables_config:
    crear_boxplot_impacto(df_estandarizado, columna_principal, nombre_principal, 
                         color_principal, variables_config)

print("\n✓ Todos los gráficos de impacto han sido generados (con datos estandarizados)")

# ================================================================================
# RANDOM FOREST - IMPORTANCIA DE VARIABLES
# ================================================================================

def entrenar_random_forest(df, columnas_numericas, n_estimators=100, max_depth=10, test_size=0.2, random_state=42):
    """
    Entrena modelos de Random Forest para cada variable como target y 
    calcula métricas de importancia.
    
    Parámetros:
    - df: DataFrame con los datos (sin estandarizar)
    - columnas_numericas: lista de nombres de columnas a analizar
    - n_estimators: número de árboles en el bosque (default: 100)
    - max_depth: profundidad máxima de cada árbol (default: 10)
    - test_size: proporción de datos para test (default: 0.2)
    - random_state: semilla para reproducibilidad (default: 42)
    
    Retorna:
    - df_resultados: DataFrame con métricas (R², RMSE, MAE) por variable
    - importancia_matriz: DataFrame con importancia de cada feature para predecir cada target
    - importancia_global: Serie con importancia acumulada de cada variable
    """
    from sklearn.ensemble import RandomForestRegressor
    from sklearn.model_selection import train_test_split
    from sklearn.metrics import r2_score, mean_absolute_error, mean_squared_error
    import numpy as np
    
    df_rf = df[columnas_numericas].copy()
    
    # Diccionario para almacenar resultados
    resultados_rf = {
        'variable': [],
        'r2': [],
        'rmse': [],
        'mae': [],
        'importancia_promedio': []
    }
    
    importancia_matriz = pd.DataFrame(index=columnas_numericas, columns=columnas_numericas, dtype=float)
    
    print("\nEntrenando Random Forest para cada variable como target...\n")
    
    # Para cada variable como target
    for target in columnas_numericas:
        print(f"→ Prediciendo: {target.upper()}")
        
        # Separar features (X) y target (y)
        features = [col for col in columnas_numericas if col != target]
        X = df_rf[features]
        y = df_rf[target]
        
        # Split train/test
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=test_size, random_state=random_state)
        
        # Entrenar Random Forest
        rf_model = RandomForestRegressor(
            n_estimators=n_estimators,
            max_depth=max_depth,
            random_state=random_state,
            n_jobs=-1
        )
        rf_model.fit(X_train, y_train)
        
        # Predicciones
        y_pred = rf_model.predict(X_test)
        
        # Métricas de error
        r2 = r2_score(y_test, y_pred)
        rmse = np.sqrt(mean_squared_error(y_test, y_pred))
        mae = mean_absolute_error(y_test, y_pred)
        
        # Importancia de features
        importancias = rf_model.feature_importances_
        importancia_promedio = np.mean(importancias)
        
        # Guardar resultados
        resultados_rf['variable'].append(target)
        resultados_rf['r2'].append(r2)
        resultados_rf['rmse'].append(rmse)
        resultados_rf['mae'].append(mae)
        resultados_rf['importancia_promedio'].append(importancia_promedio)
        
        # Guardar importancias en matriz
        for i, feature in enumerate(features):
            importancia_matriz.loc[feature, target] = importancias[i]
        
        print(f"  R² = {r2:.3f} | RMSE = {rmse:.3f} | MAE = {mae:.3f}")
        print(f"  Importancias de features: {dict(zip(features, importancias.round(3)))}")
        print()
    
    # Convertir resultados a DataFrame
    df_resultados = pd.DataFrame(resultados_rf)
    df_resultados = df_resultados.sort_values('r2', ascending=False)
    
    # Calcular importancia global
    importancia_matriz_float = importancia_matriz.astype(float)
    importancia_global = importancia_matriz_float.sum(axis=1).sort_values(ascending=False)
    
    return df_resultados, importancia_matriz_float, importancia_global


def crear_graficos_random_forest(df_resultados, importancia_matriz, importancia_global, carpeta_salida='images/random_forest'):
    """
    Crea los gráficos de visualización de resultados de Random Forest.
    
    Parámetros:
    - df_resultados: DataFrame con métricas por variable
    - importancia_matriz: DataFrame con matriz de importancias
    - importancia_global: Serie con importancia acumulada
    - carpeta_salida: carpeta donde guardar los gráficos
    
    Retorna:
    - lista de nombres de archivos guardados
    """
    import numpy as np
    
    os.makedirs(carpeta_salida, exist_ok=True)
    archivos_guardados = []
    
    # Gráfico 1: Predictibilidad (R²) de cada variable
    fig, ax = plt.subplots(figsize=(10, 6))
    colores = ['#2ecc71' if r2 > 0.7 else '#f39c12' if r2 > 0.5 else '#e74c3c' 
               for r2 in df_resultados['r2']]
    bars = ax.barh(df_resultados['variable'], df_resultados['r2'], color=colores, edgecolor='black')
    ax.set_xlabel('R² Score', fontsize=12, fontweight='bold')
    ax.set_ylabel('Variable', fontsize=12, fontweight='bold')
    ax.set_title('Predictibilidad de Variables usando Random Forest\n(qué tan bien se puede predecir cada variable)', 
                 fontsize=14, fontweight='bold')
    ax.set_xlim(0, 1)
    ax.axvline(0.7, color='gray', linestyle='--', linewidth=1, alpha=0.5, label='R²=0.7 (bueno)')
    ax.axvline(0.5, color='gray', linestyle=':', linewidth=1, alpha=0.5, label='R²=0.5 (regular)')
    ax.legend(loc='lower right')
    ax.grid(axis='x', alpha=0.3)
    
    for i, (var, r2) in enumerate(zip(df_resultados['variable'], df_resultados['r2'])):
        ax.text(r2 + 0.02, i, f'{r2:.3f}', va='center', fontsize=10, fontweight='bold')
    
    plt.tight_layout()
    archivo1 = f'{carpeta_salida}/predictibilidad_variables.png'
    plt.savefig(archivo1, dpi=300, bbox_inches='tight')
    print(f"\n✓ Gráfico guardado: {archivo1}")
    archivos_guardados.append(archivo1)
    plt.close()
    
    # Gráfico 2: Importancia global de cada variable
    fig, ax = plt.subplots(figsize=(10, 6))
    colores = plt.cm.viridis(np.linspace(0.3, 0.9, len(importancia_global)))
    bars = ax.barh(importancia_global.index, importancia_global.values, color=colores, edgecolor='black')
    ax.set_xlabel('Importancia Acumulada', fontsize=12, fontweight='bold')
    ax.set_ylabel('Variable', fontsize=12, fontweight='bold')
    ax.set_title('Ranking de Importancia de Variables\n(capacidad predictora sobre otras variables)', 
                 fontsize=14, fontweight='bold')
    ax.grid(axis='x', alpha=0.3)
    
    for i, (var, imp) in enumerate(importancia_global.items()):
        ax.text(imp + 0.01, i, f'{imp:.3f}', va='center', fontsize=10, fontweight='bold')
    
    plt.tight_layout()
    archivo2 = f'{carpeta_salida}/importancia_global.png'
    plt.savefig(archivo2, dpi=300, bbox_inches='tight')
    print(f"✓ Gráfico guardado: {archivo2}")
    archivos_guardados.append(archivo2)
    plt.close()
    
    # Gráfico 3: Matriz de importancia (heatmap)
    fig, ax = plt.subplots(figsize=(10, 8))
    sns.heatmap(importancia_matriz, annot=True, fmt='.3f', cmap='YlOrRd', 
                cbar_kws={'label': 'Importancia'}, linewidths=0.5, linecolor='black',
                vmin=0, vmax=0.4, ax=ax)
    ax.set_xlabel('Variable Predicha (Target)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Variable Predictora (Feature)', fontsize=12, fontweight='bold')
    ax.set_title('Matriz de Importancia de Variables en Random Forest\n(fila predice columna)', 
                 fontsize=14, fontweight='bold')
    plt.tight_layout()
    archivo3 = f'{carpeta_salida}/matriz_importancia.png'
    plt.savefig(archivo3, dpi=300, bbox_inches='tight')
    print(f"✓ Gráfico guardado: {archivo3}")
    archivos_guardados.append(archivo3)
    plt.close()
    
    return archivos_guardados


def imprimir_interpretacion_rf(df_resultados, importancia_global):
    """
    Imprime la interpretación de los resultados de Random Forest.
    
    Parámetros:
    - df_resultados: DataFrame con métricas por variable
    - importancia_global: Serie con importancia acumulada
    """
    print("\n" + "="*60)
    print("INTERPRETACIÓN DE RESULTADOS")
    print("="*60)
    print(f"""
1. VARIABLE MÁS IMPORTANTE (mayor capacidad predictora):
   -> {importancia_global.index[0].upper()} (importancia: {importancia_global.iloc[0]:.3f})
   
2. VARIABLE MÁS PREDECIBLE (mejor R²):
   -> {df_resultados.iloc[0]['variable'].upper()} (R² = {df_resultados.iloc[0]['r2']:.3f})
   
3. VARIABLE MENOS PREDECIBLE (peor R²):
   -> {df_resultados.iloc[-1]['variable'].upper()} (R² = {df_resultados.iloc[-1]['r2']:.3f})

Interpretación:
- R² > 0.7: Muy predecible (alta relación con otras variables)
- R² 0.5-0.7: Moderadamente predecible
- R² < 0.5: Difícil de predecir (comportamiento más independiente)

Una variable IMPORTANTE predice bien otras variables.
Una variable PREDECIBLE es influenciada fuertemente por otras.
""")


# Ejecutar análisis de Random Forest
print("\n" + "="*60)
print("RANDOM FOREST - ANÁLISIS DE IMPORTANCIA DE VARIABLES")
print("="*60)

columnas_numericas = ['temperatura', 'ph', 'turbidez', 'oxigeno', 'luz']

# Entrenar modelos
df_resultados, importancia_matriz, importancia_global = entrenar_random_forest(df, columnas_numericas)

# Mostrar resultados
print("\n" + "="*60)
print("RESULTADOS - PREDICTIBILIDAD DE CADA VARIABLE")
print("="*60)
print(df_resultados.to_string(index=False))

print("\n" + "="*60)
print("RANKING GLOBAL - IMPORTANCIA COMO FEATURE PREDICTORA")
print("="*60)
print("(Suma de importancias cuando la variable se usa para predecir otras)")
print()
for i, (var, imp) in enumerate(importancia_global.items(), 1):
    print(f"{i}. {var.upper()}: {imp:.3f}")

# Crear gráficos
crear_graficos_random_forest(df_resultados, importancia_matriz, importancia_global)

# Imprimir interpretación
imprimir_interpretacion_rf(df_resultados, importancia_global)

print("\n✓ Análisis de Random Forest completado")
