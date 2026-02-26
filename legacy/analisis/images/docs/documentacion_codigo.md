# Documentación del Código - Análisis de Sensores TACHI

## Descripción General

Este script realiza un análisis completo de datos de sensores de calidad del agua (ESP32) del proyecto TACHI. Incluye limpieza de datos, análisis estadístico, visualizaciones y modelado predictivo con Random Forest.

---

## Estructura del Código

### 1. Carga y Limpieza de Datos

#### Carga del Dataset
```python
df = pd.read_excel('Sensores TACHI.xlsx')
df = df.iloc[:, :7]  # Mantener primeras 7 columnas
```

**Columnas:**
- `tiempo`: Timestamp de la medición
- `id_dispositivo`: Identificador del sensor (esp32_01, esp32_02, esp32_03)
- `temperatura`: Temperatura del agua en °C
- `ph`: Nivel de pH (0-14)
- `turbidez`: Turbidez en NTU (Nephelometric Turbidity Units)
- `oxigeno`: Oxígeno disuelto en mg/L
- `luz`: Intensidad lumínica en Lux

#### Limpieza de Datos Anormales

**Valores -1 (fallo del sensor):**
```python
df = df[(df != -1).all(axis=1)]
```
Elimina todas las filas donde alguna columna tiene valor -1, que indica error de lectura del sensor.

**Rangos físicamente válidos:**
- Temperatura: [-10°C, 40°C] - Rango razonable para agua
- pH: [0, 14] - Rango químico válido
- Oxígeno: [0, 20 mg/L] - Rango típico en agua
- Turbidez y Luz: ≥ 0 - No pueden ser negativos

**Eliminación de valores nulos:**
```python
df = df.dropna(subset=columnas_numericas)
```
Se eliminan todas las filas con valores faltantes en lugar de imputarlos, para mantener integridad de los datos.

---

### 2. Tratamiento de Outliers y Transformaciones

#### Tratamiento de LUZ

**Problema:** 
- Luz tiene variabilidad extrema (CV=464.5%)
- Rango: 0.01 - 39,662 Lux
- Valores extremos distorsionan visualizaciones

**Solución:**
```python
percentil_99_luz = df['luz'].quantile(0.99)
df = df[df['luz'] <= percentil_99_luz]
df['luz_original'] = df['luz'].copy()
df['luz'] = np.log1p(df['luz'])
```

1. **Eliminar percentil 99:** Remueve ~10 valores extremos (>3,104 Lux)
2. **Transformación logarítmica:** `log(x + 1)` comprime la escala
   - Razón: Datos de luminosidad siguen distribución log-normal
   - Mejora normalidad y reduce impacto de outliers

#### Tratamiento de TURBIDEZ

**Problema:**
- 81.9% de datos concentrados en 90-100 NTU
- 6.3% de valores bajos (<50 NTU) actúan como outliers
- Distribución altamente sesgada

**Solución:**
```python
percentil_5_turbidez = df['turbidez'].quantile(0.05)
df = df[df['turbidez'] >= percentil_5_turbidez]
df['turbidez_original'] = df['turbidez'].copy()
df['turbidez'] = np.log1p(df['turbidez'])
```

1. **Eliminar percentil 5:** Remueve valores anormalmente bajos (<40.7 NTU)
2. **Transformación logarítmica:** Normaliza distribución para análisis estadístico

**Justificación:** Los valores bajos probablemente indican mal funcionamiento del sensor o condiciones anómalas que distorsionan el análisis general.

---

### 3. Estandarización (Z-Score)

```python
from sklearn.preprocessing import StandardScaler
scaler = StandardScaler()
df_estandarizado[columnas_numericas] = scaler.fit_transform(df[columnas_numericas])
```

**Propósito:**
- Transforma todas las variables a media=0 y desviación estándar=1
- Permite comparar variables con diferentes unidades y escalas
- Necesario para análisis de covarianza y boxplots comparativos

**Fórmula:** `z = (x - μ) / σ`
- `x`: valor original
- `μ`: media de la variable
- `σ`: desviación estándar

---

## Funciones Principales

### `crear_analisis_temporal_histograma()`

```python
def crear_analisis_temporal_histograma(df, df_sorted, columna, nombre_variable, 
                                       color, unidad, referencia=None)
```

**Propósito:** Crea visualización combinada de serie temporal e histograma para una variable.

**Parámetros:**
- `df`: DataFrame con datos completos
- `df_sorted`: DataFrame ordenado cronológicamente
- `columna`: Nombre de la columna a analizar
- `nombre_variable`: Nombre descriptivo para títulos
- `color`: Color para los gráficos
- `unidad`: Unidad de medida (ej: "°C", "mg/L")
- `referencia`: Valor de referencia opcional (ej: pH=7 neutro)

**Funcionamiento:**
1. **Serie Temporal (subplot superior):**
   - Línea temporal de la variable
   - Media (línea azul punteada)
   - ±1 desviación estándar (líneas verdes)
   - Línea de referencia opcional (gris)
   - Formato de fecha cada 5 días

2. **Histograma (subplot inferior):**
   - Distribución de frecuencias (50 bins)
   - Líneas de media (azul) y mediana (roja)
   - Estadísticas descriptivas en texto

**Salida:** Guarda imagen PNG en `images/histogramas y ST/`

**Uso:**
```python
crear_analisis_temporal_histograma(df, df_sorted, 'temperatura', 
                                   'Temperatura', 'orangered', '°C')
```

---

### `crear_categorias()`

```python
def crear_categorias(serie)
```

**Propósito:** Divide una serie numérica en 3 categorías basadas en percentiles.

**Funcionamiento:**
- Calcula percentil 33 y 66
- Crea categorías:
  - **Bajo:** valores < percentil 33
  - **Medio:** percentil 33 ≤ valores < percentil 66
  - **Alto:** valores ≥ percentil 66

**Retorna:** Serie categórica con etiquetas 'Bajo', 'Medio', 'Alto'

**Uso:** Preparar datos para boxplots comparativos.

---

### `crear_boxplot_impacto()`

```python
def crear_boxplot_impacto(df_estandarizado, columna_principal, nombre_principal, 
                         color_principal, variables_config, 
                         carpeta_salida='images/cajas y bigotes')
```

**Propósito:** Visualiza cómo una variable (categorizada) afecta a las demás variables del sistema.

**Parámetros:**
- `df_estandarizado`: DataFrame con datos z-score
- `columna_principal`: Variable independiente a categorizar
- `nombre_principal`: Nombre descriptivo
- `color_principal`: Color (no usado actualmente)
- `variables_config`: Lista de tuplas `(columna, nombre, color)` de todas las variables
- `carpeta_salida`: Directorio para guardar gráficos

**Funcionamiento:**
1. Categoriza la variable principal en Bajo/Medio/Alto
2. Crea figura 2x2 con 4 subplots (las otras 4 variables)
3. Para cada subplot:
   - Boxplot de valores z-score por categoría
   - Mediana (línea roja), media (diamante azul)
   - Conteo de observaciones por categoría
   - Línea de referencia en z=0 (media global)

**Interpretación de Boxplots:**
- **Centro del boxplot desplazado:** Indica correlación con la variable principal
- **Cajas amplias:** Alta variabilidad dentro de la categoría
- **Outliers:** Valores atípicos (>1.5 * IQR desde cuartiles)

**Salida:** Archivo PNG con nombre `{columna_principal}_impacto.png`

---

### `entrenar_random_forest()`

```python
def entrenar_random_forest(df, columnas_numericas, n_estimators=100, 
                          max_depth=10, test_size=0.2, random_state=42)
```

**Propósito:** Entrena modelos de Random Forest para cada variable como target y calcula importancia de features.

**Parámetros:**
- `df`: DataFrame con datos originales (sin estandarizar)
- `columnas_numericas`: Lista de variables a analizar
- `n_estimators`: Número de árboles en el bosque (default: 100)
- `max_depth`: Profundidad máxima de cada árbol (default: 10)
- `test_size`: Proporción para conjunto de prueba (default: 0.2 = 80/20 split)
- `random_state`: Semilla para reproducibilidad (default: 42)

**Algoritmo:**
1. Para cada variable como target:
   - Separar features (X) y target (y)
   - Split train/test estratificado
   - Entrenar RandomForestRegressor
   - Predecir en conjunto de test
   - Calcular métricas: R², RMSE, MAE
   - Extraer importancia de cada feature

2. Construir matriz de importancia (NxN)
   - Filas: features
   - Columnas: targets
   - Valores: importancia de feature para predecir target

3. Calcular importancia global (suma por fila)

**Retorna:**
- `df_resultados`: DataFrame con métricas por variable
- `importancia_matriz`: DataFrame NxN con importancias
- `importancia_global`: Serie con importancia acumulada

**Métricas:**
- **R² (Coeficiente de Determinación):** Proporción de varianza explicada (0-1)
  - R²=1: Predicción perfecta
  - R²=0: Modelo no mejor que media
- **RMSE (Root Mean Square Error):** Error promedio en unidades originales
- **MAE (Mean Absolute Error):** Error absoluto promedio

**Importancia de Features:**
Random Forest calcula importancia por reducción de impureza (Gini):
- Mide cuánto mejora la predicción al incluir cada variable
- Valores normalizados que suman 1 para cada modelo

---

### `crear_graficos_random_forest()`

```python
def crear_graficos_random_forest(df_resultados, importancia_matriz, 
                                importancia_global, 
                                carpeta_salida='images/random_forest')
```

**Propósito:** Genera visualizaciones de los resultados de Random Forest.

**Gráficos generados:**

1. **Predictibilidad (R²):**
   - Gráfico de barras horizontales
   - Colores por nivel: Verde (>0.7), Naranja (0.5-0.7), Rojo (<0.5)
   - Líneas de referencia en R²=0.7 y R²=0.5
   - Muestra qué tan bien se puede predecir cada variable

2. **Importancia Global:**
   - Barras horizontales con gradiente viridis
   - Suma de importancias cuando variable es feature
   - Ranking de capacidad predictora

3. **Matriz de Importancia (Heatmap):**
   - Mapa de calor YlOrRd (amarillo-naranja-rojo)
   - Anotaciones con valores numéricos (3 decimales)
   - Fila X predice Columna Y

**Salida:** Lista de nombres de archivos guardados (300 dpi, alta calidad)

---

### `imprimir_interpretacion_rf()`

```python
def imprimir_interpretacion_rf(df_resultados, importancia_global)
```

**Propósito:** Imprime resumen interpretativo de resultados de Random Forest.

**Información mostrada:**
1. Variable más importante (mayor importancia global)
2. Variable más predecible (mayor R²)
3. Variable menos predecible (menor R²)
4. Guía de interpretación de R²
5. Diferencia entre importancia y predictibilidad

**Conceptos clave:**
- **Variable IMPORTANTE:** Predice bien otras variables (buen feature)
- **Variable PREDECIBLE:** Es influenciada fuertemente por otras (buen target)
- Una variable puede ser importante pero poco predecible (comportamiento independiente influyente)

---

## Pipeline de Análisis Completo

### Orden de Ejecución:

```
1. Carga de datos
   ↓
2. Limpieza (valores -1, rangos inválidos, nulos)
   ↓
3. Tratamiento de outliers (luz y turbidez)
   ↓
4. Estandarización z-score
   ↓
5. Análisis de covarianza/correlación
   ↓
6. Visualizaciones temporales e histogramas (5 variables)
   ↓
7. Boxplots de impacto (5 gráficos)
   ↓
8. Random Forest (entrenamiento + visualizaciones)
   ↓
9. Interpretación y conclusiones
```

---

## Dependencias Requeridas

```python
pandas>=1.3.0           # Manipulación de datos
matplotlib>=3.4.0       # Visualizaciones
seaborn>=0.11.0         # Heatmaps y gráficos estadísticos
scikit-learn>=0.24.0    # Random Forest y métricas
numpy>=1.21.0           # Operaciones numéricas
openpyxl>=3.0.0         # Lectura de archivos Excel
```

---

## Estructura de Archivos Generados

```
data/
├── Sensores TACHI.xlsx         # Dataset original
├── analisis.py                 # Script principal
├── docs/
│   ├── documentacion_codigo.md    # Este archivo
│   └── documentacion_graficos.md  # Documentación de gráficos
└── images/
    ├── histogramas y ST/
    │   ├── temperatura_analisis.png
    │   ├── ph_analisis.png
    │   ├── turbidez_analisis.png
    │   ├── oxigeno_analisis.png
    │   └── luz_analisis.png
    ├── cajas y bigotes/
    │   ├── temperatura_impacto.png
    │   ├── ph_impacto.png
    │   ├── turbidez_impacto.png
    │   ├── oxigeno_impacto.png
    │   └── luz_impacto.png
    ├── random_forest/
    │   ├── predictibilidad_variables.png
    │   ├── importancia_global.png
    │   └── matriz_importancia.png
    └── matriz_covarianza_estandarizada.png
```

---

## Buenas Prácticas Implementadas

### 1. Documentación
- Docstrings completos en todas las funciones
- Comentarios explicativos en código complejo
- Nombres de variables descriptivos

### 2. Modularización
- Funciones reutilizables con responsabilidad única
- Parámetros configurables
- Separación de lógica y presentación

### 3. Reproducibilidad
- `random_state=42` en Random Forest
- Transformaciones documentadas con valores originales guardados
- Pipeline secuencial claro

### 4. Calidad de Visualizaciones
- Resolución 300 dpi (calidad publicación)
- Colores consistentes por variable
- Etiquetas y títulos descriptivos
- Leyendas informativas

### 5. Gestión de Datos
- Validación de rangos físicos
- Tratamiento explícito de outliers
- Preservación de datos originales

---

## Notas Técnicas

### Estandarización vs Normalización
- **Estandarización (z-score):** Usada en este análisis
  - Centrada en 0, escala por desviación estándar
  - Mantiene información de outliers
  - Mejor para datos con distribución normal
  
- **Normalización (min-max):** No usada
  - Escala a rango [0,1]
  - Sensible a outliers extremos

### Random Forest: Hiperparámetros
- **n_estimators=100:** Equilibrio entre precisión y tiempo de cómputo
- **max_depth=10:** Evita overfitting mientras captura patrones complejos
- **n_jobs=-1:** Usa todos los CPUs disponibles (paralelización)

### Transformación Logarítmica
- **log1p (log(x+1)):** Maneja valores cercanos a 0 sin error
- Adecuada para:
  - Datos con distribución log-normal (luz, concentraciones)
  - Alta variabilidad (CV > 100%)
  - Outliers extremos

---

## Posibles Extensiones

### Análisis Adicionales Sugeridos:
1. **PCA (Principal Component Analysis):** Reducción de dimensionalidad
2. **Clustering:** Identificar grupos de mediciones similares
3. **Detección de anomalías:** Isolation Forest, Local Outlier Factor
4. **Series temporales:** ARIMA, Prophet para predicción futura
5. **Análisis por dispositivo:** Comparar calibraciones entre esp32_01, esp32_02, esp32_03
6. **Análisis de eventos:** Detectar eventos críticos de pH, oxígeno bajo, etc.

### Mejoras de Código:
1. Argumentos de línea de comandos (argparse)
2. Archivo de configuración (YAML/JSON)
3. Logging estructurado
4. Tests unitarios
5. Paralelización de generación de gráficos
6. Dashboard interactivo (Streamlit/Dash)

---

## Autor y Versión

**Versión:** 1.0  
**Fecha:** Enero 2026  
**Proyecto:** Análisis de Sensores TACHI - Calidad del Agua  
**Lenguaje:** Python 3.8+  

---

## Referencias

1. Breiman, L. (2001). Random Forests. Machine Learning, 45(1), 5-32.
2. Pedregosa et al. (2011). Scikit-learn: Machine Learning in Python. JMLR 12, pp. 2825-2830.
3. ISO 7027-1:2016 - Water quality — Determination of turbidity
4. WHO (2017). Guidelines for Drinking-water Quality, 4th edition
