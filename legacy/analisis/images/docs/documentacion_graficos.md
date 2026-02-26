# Documentación de Gráficos - Análisis de Sensores TACHI

## Índice de Gráficos

1. **Series Temporales e Histogramas** (5 gráficos)
   - Temperatura
   - pH
   - Turbidez
   - Oxígeno Disuelto
   - Luz

2. **Matriz de Covarianza/Correlación Estandarizada** (1 gráfico)

3. **Análisis de Impacto - Boxplots** (5 gráficos)
   - Impacto de Temperatura
   - Impacto de pH
   - Impacto de Turbidez
   - Impacto de Oxígeno
   - Impacto de Luz

4. **Random Forest - Análisis de Importancia** (3 gráficos)
   - Predictibilidad de Variables (R²)
   - Ranking de Importancia Global
   - Matriz de Importancia

---

# 1. SERIES TEMPORALES E HISTOGRAMAS

## 1.1. Temperatura

**Archivo:** `images/histogramas y ST/temperatura_analisis.png`

### Explicación

**Serie Temporal (Gráfico Superior):**
- Muestra la evolución de la temperatura del agua durante el período de medición
- Línea azul punteada: Media = 21.52°C
- Líneas verdes punteadas: ±1 desviación estándar (±2.04°C)
- Rango temporal: Aproximadamente 2 meses de mediciones

**Histograma (Gráfico Inferior):**
- Distribución de frecuencias de temperatura
- Línea azul: Media (21.52°C)
- Línea roja: Mediana (21.70°C)
- Distribución aproximadamente normal (campana de Gauss)

### Conclusiones

1. **Estabilidad térmica:** La temperatura se mantiene relativamente estable con CV=9.5% (baja variabilidad)
2. **Rango operativo:** 16.75°C - 28.94°C, dentro de rangos seguros para vida acuática
3. **Patrón temporal:** No se observan tendencias de calentamiento/enfriamiento sostenido, sugiere condiciones ambientales estables
4. **Distribución normal:** Los datos siguen una distribución gaussiana, indicando comportamiento natural sin anomalías sistémicas
5. **Media vs Mediana:** Valores muy cercanos (21.52 vs 21.70) confirman simetría de la distribución

**Implicación práctica:** Las condiciones térmicas del agua son apropiadas y estables para ecosistemas acuáticos.

---

## 1.2. pH

**Archivo:** `images/histogramas y ST/ph_analisis.png`

### Explicación

**Serie Temporal (Gráfico Superior):**
- Evolución del pH durante el período de medición
- Línea gris: pH neutro (7.0) como referencia
- Línea azul punteada: Media = 8.06
- La mayoría de mediciones están por encima de 7 (alcalino)
- Se observan dos "zonas" de medición claramente diferenciadas

**Histograma (Gráfico Inferior):**
- **Distribución bimodal:** Dos picos evidentes
  - Pico principal: ~8.0 (mayoría de mediciones)
  - Pico secundario: ~8.9
- Esto sugiere dos poblaciones diferentes de mediciones

### Conclusiones

1. **Agua ligeramente alcalina:** Media de 8.06, dentro de rango aceptable para agua natural (6.5-9.0)
2. **Bimodalidad detectada:** Los dos picos indican dos condiciones diferentes:
   - Posiblemente dos sensores con calibraciones diferentes (esp32_01 vs esp32_03)
   - O dos condiciones ambientales distintas en momentos diferentes
3. **Eventos críticos:** Se detectaron 87 eventos con pH >9.5 o <6.0 (2.7% del total)
   - 394 eventos con pH >9.0 (12.3%) requieren atención
   - 24 eventos con pH <6.5 (0.7%)
4. **Baja variabilidad (CV=9.0%):** Aunque estadísticamente estable, cualquier desviación es biológicamente significativa
5. **Importancia práctica vs estadística:** pH tiene baja variabilidad estadística pero ALTA importancia práctica para calidad del agua

**Implicación práctica:** Sistema de monitoreo debe incluir alertas automáticas para pH fuera de rango 6.5-9.0, independientemente de modelos predictivos.

---

## 1.3. Turbidez

**Archivo:** `images/histogramas y ST/turbidez_analisis.png`

**Nota:** Gráfico muestra datos transformados log(turbidez+1) después de eliminar percentil 5.

### Explicación

**Serie Temporal (Gráfico Superior):**
- Después de transformación logarítmica: rango 3.73-4.62
- Equivale a ~40-100 NTU en escala original
- Valores muy concentrados en el extremo superior
- Algunos períodos con valores notablemente más bajos

**Histograma (Gráfico Inferior):**
- Distribución fuertemente sesgada hacia valores altos
- La mayoría de mediciones en log(turbidez) ≈ 4.5-4.6
- Corresponde a ~90-100 NTU (cercano al límite del sensor)

### Conclusiones

1. **Alta turbidez predominante:** 81.9% de lecturas entre 90-100 NTU
   - Sugiere agua constantemente turbia
   - Posible saturación del sensor en valores máximos
2. **Outliers bajos (6.3%):** Valores <50 NTU probablemente indican:
   - Eventos de limpieza/clarificación del agua
   - Mal funcionamiento temporal del sensor
   - Cambios ambientales significativos
3. **Transformación necesaria:** Datos originales muy sesgados requirieron transformación log para análisis estadístico
4. **Variabilidad moderada (CV=23.9%):** Después de transformación
5. **Relación con oxígeno:** Alta turbidez correlaciona negativamente con oxígeno disuelto (-0.485)

**Implicación práctica:** Agua con turbidez consistentemente alta puede indicar contaminación por sedimentos, requiere investigación de la fuente.

---

## 1.4. Oxígeno Disuelto

**Archivo:** `images/histogramas y ST/oxigeno_analisis.png`

### Explicación

**Serie Temporal (Gráfico Superior):**
- Alta variabilidad en las mediciones
- Rango: 0.0 - 20.0 mg/L (rango completo del sensor)
- Media: 8.72 mg/L
- Desviación estándar alta: ±4.86 mg/L
- Fluctuaciones significativas en períodos cortos

**Histograma (Gráfico Inferior):**
- Distribución relativamente uniforme
- Ligero sesgo hacia valores altos
- Media (8.72) y mediana cercanas pero con dispersión considerable

### Conclusiones

1. **Alta variabilidad (CV=55.7%):** La variable más variable del sistema
   - Refleja dinámicas naturales del ecosistema acuático
   - Influenciado por múltiples factores: temperatura, fotosíntesis, respiración, turbidez
2. **Niveles críticos ocasionales:** Valores cercanos a 0 mg/L son letales para peces
   - Se observan algunos eventos con oxígeno muy bajo
   - Requiere sistema de alerta inmediata
3. **Media aceptable (8.72 mg/L):** Por encima del mínimo recomendado (5-6 mg/L para vida acuática)
4. **Fuerte correlación con turbidez (-0.485):** A mayor turbidez, menor oxígeno
   - Probablemente por reducción de fotosíntesis (menos luz penetra agua turbia)
5. **Variable más predecible (R²=0.926):** Fuertemente influenciada por otras variables del sistema

**Implicación práctica:** Oxígeno disuelto es indicador clave de salud del ecosistema y debe monitorearse continuamente con alertas de umbral crítico (<5 mg/L).

---

## 1.5. Luz (Intensidad Lumínica)

**Archivo:** `images/histogramas y ST/luz_analisis.png`

**Nota:** Gráfico muestra datos transformados log(luz+1) después de eliminar percentil 99.

### Explicación

**Serie Temporal (Gráfico Superior):**
- Después de transformación: rango 0.01-8.00
- Equivale a ~0-3,000 Lux en escala original
- Patrones cíclicos visibles (probablemente día/noche)
- Variabilidad extrema antes de transformación (CV=464.5%)

**Histograma (Gráfico Inferior):**
- Después de log-transformación, distribución más manejable
- Mediana muy baja (10.49 Lux) en datos originales
- 50% de mediciones < 10.49 Lux

### Conclusiones

1. **Variabilidad extrema (CV=464.5%):** La más variable de todas las variables
   - Rango original: 0.01 - 39,662 Lux
   - Refleja ciclos día/noche, nubosidad, profundidad, turbidez del agua
2. **Outliers extremos:** 0.3% de valores >5,000 Lux
   - Probablemente mediciones con sensor fuera del agua o condiciones excepcionales
   - Removidos percentil 99 (>3,104 Lux) para análisis
3. **Distribución log-normal:** Típica de mediciones de luminosidad
   - Transformación logarítmica necesaria para análisis estadístico
4. **Relación con turbidez y oxígeno:** 
   - Luz afecta fotosíntesis → producción de oxígeno
   - Turbidez bloquea luz → reduce fotosíntesis
5. **Predictibilidad moderada (R²=0.792):** Influenciada por otras variables pero con componente independiente (ciclos día/noche)

**Implicación práctica:** Patrones de luz ayudan a entender ciclos fotosintéticos y su impacto en oxígeno disuelto.

---

# 2. MATRIZ DE COVARIANZA/CORRELACIÓN ESTANDARIZADA

**Archivo:** `images/matriz_covarianza_estandarizada.png`

### Explicación

Heatmap que muestra la covarianza entre pares de variables usando datos estandarizados (z-score). En datos estandarizados, covarianza = correlación.

**Escala de colores:**
- Azul oscuro: Correlación negativa fuerte (~ -0.5)
- Blanco/Amarillo: Correlación cercana a 0
- Rojo: Correlación positiva fuerte (~ +0.4)

**Diagonal principal:** Siempre ≈1.0 (variable consigo misma)

**Valores clave:**
- **Turbidez ↔ Oxígeno: -0.508** (rojo oscuro) - Correlación negativa más fuerte
- **Temperatura ↔ Oxígeno: -0.361** - Correlación negativa moderada
- **Temperatura ↔ Turbidez: +0.363** - Correlación positiva moderada
- **pH ↔ Oxígeno: -0.190** - Correlación negativa débil
- **pH con otras:** Correlaciones muy débiles (<0.1)

### Conclusiones

1. **Turbidez-Oxígeno: Relación más fuerte del sistema (-0.508)**
   - A mayor turbidez, menor oxígeno disuelto
   - Mecanismo: Agua turbia bloquea luz → reduce fotosíntesis → menos oxígeno producido
   - Esta es la relación predictiva más importante

2. **Temperatura-Oxígeno: Relación inversa (-0.361)**
   - A mayor temperatura, menor oxígeno disuelto
   - Física: Agua caliente retiene menos gases disueltos (ley de Henry)
   - Biología: Mayor temperatura aumenta metabolismo y consumo de O₂

3. **Temperatura-Turbidez: Relación positiva (+0.363)**
   - Correlación inesperada, posibles causas:
   - Actividad biológica aumenta con temperatura → más sedimentos suspendidos
   - Correlación espuria: ambas afectadas por tercera variable no medida

4. **pH: Variable más independiente**
   - Correlaciones débiles con todas las demás (<0.2)
   - Baja variabilidad (CV=9.0%) limita correlaciones estadísticas
   - No significa que no sea importante, solo que varía independientemente

5. **Luz: Correlaciones moderadas**
   - Modesta relación con temperatura y turbidez
   - Ciclos día/noche agregan variabilidad independiente

**Implicación práctica:** El sistema está dominado por la relación turbidez-oxígeno. Reducir turbidez debería aumentar oxígeno disuelto.

---

# 3. ANÁLISIS DE IMPACTO - BOXPLOTS

Los boxplots muestran cómo cada variable (categorizada en Bajo/Medio/Alto) afecta a las demás variables. Datos estandarizados (z-score).

**Interpretación de Boxplots:**
- **Caja:** Rango intercuartílico (Q1-Q3, 50% central de datos)
- **Línea roja en caja:** Mediana
- **Diamante azul:** Media
- **Bigotes:** Rango ~1.5×IQR desde cuartiles
- **Puntos fuera:** Outliers
- **Línea gris punteada (z=0):** Media global

**Desplazamiento de cajas indica correlación:**
- Cajas ascendentes (Bajo→Medio→Alto): Correlación positiva
- Cajas descendentes: Correlación negativa
- Cajas horizontales: Sin correlación

---

## 3.1. Impacto de Temperatura

**Archivo:** `images/cajas y bigotes/temperatura_impacto.png`

### Explicación

Muestra cómo temperatura (categorizada) afecta a: pH, Turbidez, Oxígeno, Luz.

**Patrones observados:**
- **Temperatura → Oxígeno:** Cajas descendentes claramente
  - Temperatura Baja → Oxígeno Alto
  - Temperatura Alta → Oxígeno Bajo
- **Temperatura → Turbidez:** Cajas ascendentes
  - Temperatura alta correlaciona con turbidez alta
- **Temperatura → pH:** Relación débil/inconsistente
- **Temperatura → Luz:** Relación débil

### Conclusiones

1. **Efecto térmico en oxígeno es claro:** Temperatura alta reduce significativamente oxígeno disuelto
   - Consistente con correlación -0.361 de la matriz
   - Mecanismo físico-químico bien establecido
   
2. **Temperatura influye en turbidez:** Posiblemente por mayor actividad biológica en agua caliente

3. **pH insensible a temperatura:** Confirma independencia de pH respecto a otras variables

4. **Temperatura como factor de control:** Aunque tiene CV bajo (9.5%), sus efectos en oxígeno son importantes

**Implicación práctica:** En períodos de alta temperatura, esperar niveles de oxígeno más bajos y necesidad de mayor vigilancia.

---

## 3.2. Impacto de pH

**Archivo:** `images/cajas y bigotes/ph_impacto.png`

### Explicación

Muestra cómo pH (categorizado) afecta a: Temperatura, Turbidez, Oxígeno, Luz.

**Patrones observados:**
- **pH → Todas:** Relaciones muy débiles
- Cajas prácticamente horizontales en todos los subplots
- Medianas y medias muy cercanas entre categorías Bajo/Medio/Alto
- Muchos outliers en todas las categorías

### Conclusiones

1. **pH no tiene efecto predictivo en otras variables:** Consistente con correlaciones bajas (<0.2) de la matriz

2. **pH es variable independiente:** Controlada por factores no capturados en este conjunto de variables
   - Posibles factores: contaminantes, actividad biológica específica, geología del sustrato

3. **Importancia práctica vs estadística:** 
   - Baja importancia estadística (ranking 3° en RF)
   - ALTA importancia práctica (87 eventos críticos detectados)
   - Debe monitorearse independientemente

4. **No es predictor útil:** Usar pH para predecir otras variables no es confiable

5. **Requiere monitoreo directo:** No se puede inferir estado de pH desde otras variables

**Implicación práctica:** pH debe tener sistema de monitoreo y alerta independiente, no puede ser predicho por modelo.

---

## 3.3. Impacto de Turbidez

**Archivo:** `images/cajas y bigotes/turbidez_impacto.png`

### Explicación

Muestra cómo turbidez (categorizada) afecta a: Temperatura, pH, Oxígeno, Luz.

**Patrones observados:**
- **Turbidez → Oxígeno:** Cajas descendentes MUY pronunciadas
  - Turbidez Baja → Oxígeno Muy Alto (z ≈ +1)
  - Turbidez Alta → Oxígeno Muy Bajo (z ≈ -0.5)
  - Efecto más fuerte observado en todos los boxplots
  
- **Turbidez → Temperatura:** Cajas ligeramente ascendentes

- **Turbidez → pH:** Relación débil

- **Turbidez → Luz:** Relación moderada descendente
  - Agua turbia bloquea luz

### Conclusiones

1. **Turbidez es el predictor más fuerte del sistema:**
   - Mayor impacto en oxígeno disuelto
   - Confirmado por importancia global más alta en Random Forest (1.780)
   
2. **Mecanismo físico claro:** Agua turbia → bloquea luz → reduce fotosíntesis → menos producción de O₂

3. **Variable clave de monitoreo:** Si solo pudieras medir una variable para inferir estado del sistema, sería turbidez

4. **Cascada de efectos:** Turbidez afecta luz, luz afecta fotosíntesis, fotosíntesis afecta oxígeno

5. **Importancia de control:** Reducir turbidez tendría mayor impacto positivo en calidad general del agua

**Implicación práctica:** Turbidez es la variable de intervención más importante. Sistemas de filtración/sedimentación mejorarían significativamente el ecosistema.

---

## 3.4. Impacto de Oxígeno

**Archivo:** `images/cajas y bigotes/oxigeno_impacto.png`

### Explicación

Muestra cómo oxígeno (categorizado) afecta a: Temperatura, pH, Turbidez, Luz.

**Patrones observados:**
- **Oxígeno → Turbidez:** Cajas descendentes fuertes
  - Oxígeno Alto → Turbidez Baja
  - Relación inversa clara
  
- **Oxígeno → Temperatura:** Cajas descendentes
  - Oxígeno alto ocurre con temperatura baja
  
- **Oxígeno → pH:** Relación débil

- **Oxígeno → Luz:** Relación débil/moderada

### Conclusiones

1. **Oxígeno es variable de respuesta (target), no predictora:**
   - R² = 0.926 (segunda más predecible)
   - Es fuertemente influenciado por turbidez y temperatura
   - No predice bien otras variables (importancia global: 1.187, 2° lugar)

2. **Indicador de salud del sistema:** Como variable de respuesta integrada
   - Refleja estado combinado de turbidez, temperatura, luz
   - Oxígeno bajo = señal de alarma de múltiples problemas

3. **Relación bidireccional con turbidez:** 
   - Turbidez alta → oxígeno bajo (causa física)
   - Oxígeno bajo → posible aumento de turbidez (procesos anaeróbicos)

4. **Variable crítica para vida acuática:** Más importante que sus correlaciones estadísticas

**Implicación práctica:** Oxígeno es el "termómetro" de la salud del agua - nivel bajo indica problemas múltiples que requieren investigación.

---

## 3.5. Impacto de Luz

**Archivo:** `images/cajas y bigotes/luz_impacto.png`

### Explicación

Muestra cómo luz (categorizada) afecta a: Temperatura, pH, Turbidez, Oxígeno.

**Patrones observados:**
- **Luz → Turbidez:** Cajas descendentes moderadas
  - Más luz corresponde con menos turbidez (agua más clara permite más luz)
  
- **Luz → Oxígeno:** Relación positiva débil/moderada
  - Más luz tiende a más oxígeno (fotosíntesis)
  
- **Luz → Temperatura:** Relación débil

- **Luz → pH:** Sin relación clara

### Conclusiones

1. **Luz tiene componente independiente fuerte:**
   - Ciclos día/noche no capturados por otras variables
   - CV=464.5% (mayor variabilidad del sistema)
   - R²=0.792 (moderadamente predecible)

2. **Relación con turbidez es bidireccional:**
   - Agua turbia bloquea luz (causa)
   - Baja luz puede aumentar sedimentación (efecto)

3. **Influencia en fotosíntesis:**
   - Luz → Oxígeno vía fotosíntesis algal
   - Pero relación debilitada por alta turbidez

4. **Importancia global moderada (0.719, 4° lugar):**
   - No es predictor fuerte de otras variables
   - Pero sí factor importante en balance del ecosistema

5. **Mediciones afectadas por posicionamiento del sensor:**
   - Valores extremos (>39K Lux) sugieren sensor parcialmente fuera del agua en ocasiones

**Implicación práctica:** Luz es importante para entender ciclos de producción de oxígeno, pero su alta variabilidad requiere promediado temporal para análisis.

---

# 4. RANDOM FOREST - ANÁLISIS DE IMPORTANCIA

## 4.1. Predictibilidad de Variables (R²)

**Archivo:** `images/random_forest/predictibilidad_variables.png`

### Explicación

Gráfico de barras horizontales mostrando R² Score de cada variable cuando se usa como target (predicha por las otras 4).

**Escala de colores:**
- Verde: R² > 0.7 (muy predecible)
- Naranja: 0.5 < R² < 0.7 (moderadamente predecible)
- Rojo: R² < 0.5 (difícil de predecir)

**Resultados:**
1. **Turbidez: R² = 0.955** (verde) - Extremadamente predecible
2. **Oxígeno: R² = 0.926** (verde) - Muy predecible
3. **Luz: R² = 0.792** (verde) - Predecible
4. **Temperatura: R² = 0.732** (verde) - Predecible
5. **pH: R² = 0.701** (verde/límite) - Moderadamente predecible

### Conclusiones

1. **Turbidez es la variable más predecible (95.5% de varianza explicada):**
   - Su comportamiento está fuertemente ligado a las otras variables
   - Modelo RF puede predecir turbidez casi perfectamente
   - Implicación: Turbidez es variable "resultado" del sistema

2. **Oxígeno también muy predecible (92.6%):**
   - Fuertemente dependiente de temperatura, turbidez, luz
   - Comportamiento determinista
   - Implicación: Se puede prever oxígeno con alta precisión

3. **pH menos predecible (70.1%):**
   - 30% de su varianza es independiente de las otras variables
   - Factores externos no medidos influyen en pH
   - Confirma necesidad de monitoreo directo

4. **Todas las variables están interconectadas:**
   - Incluso la menos predecible (pH) tiene R²>0.7
   - Sistema altamente acoplado
   - Cambios en una variable afectan a las demás

5. **Implicación para sensores:**
   - Podría reducirse frecuencia de medición de turbidez/oxígeno (predicción confiable)
   - pH requiere medición directa frecuente (baja predictibilidad relativa)

**Implicación práctica:** Sistema de sensores redundante puede optimizarse - turbidez y oxígeno son parcialmente redundantes, pero pH es irreemplazable.

---

## 4.2. Ranking de Importancia Global

**Archivo:** `images/random_forest/importancia_global.png`

### Explicación

Gráfico de barras horizontales con gradiente de color viridis mostrando la importancia acumulada de cada variable como feature predictor.

**Cálculo:** Suma de importancias cuando la variable se usa para predecir las otras 4 variables.

**Ranking:**
1. **Turbidez: 1.780** - Mejor predictor global
2. **Oxígeno: 1.187** - Predictor fuerte
3. **pH: 0.813** - Predictor moderado
4. **Luz: 0.719** - Predictor moderado-débil
5. **Temperatura: 0.502** - Predictor más débil

### Conclusiones

1. **Turbidez es la variable más influyente (1.780):**
   - Conocer turbidez da más información sobre el estado del sistema
   - Mejor variable individual para caracterizar calidad del agua
   - Si tuvieras que elegir UN sensor, sería turbidez

2. **Oxígeno segundo más importante (1.187):**
   - A pesar de ser muy predecible, también es buen predictor
   - Variable "central" del sistema (predicho y predictor)

3. **pH importancia moderada (0.813):**
   - A pesar de baja correlación, tiene poder predictivo
   - Cuando pH cambia, sí afecta a otras variables
   - Paradoja: baja variabilidad pero impactos importantes cuando varía

4. **Temperatura menos importante (0.502):**
   - Baja variabilidad (CV=9.5%) limita su poder predictivo
   - A pesar de efectos físico-químicos conocidos en oxígeno
   - Rango estrecho de temperatura reduce información

5. **Discrepancia importancia estadística vs práctica:**
   - Estadística: Turbidez > Oxígeno > pH > Luz > Temperatura
   - Práctica: pH y Oxígeno son críticos para vida acuática independiente de su poder predictivo

**Implicación práctica:** Para modelado predictivo, priorizar turbidez. Para monitoreo de seguridad, priorizar pH y oxígeno con alertas independientes.

---

## 4.3. Matriz de Importancia

**Archivo:** `images/random_forest/matriz_importancia.png`

### Explicación

Heatmap mostrando importancia de cada feature (fila) para predecir cada target (columna).

**Lectura:** Fila X predice Columna Y con importancia Z.

**Escala de colores:**
- Amarillo: Importancia baja (~0.0-0.1)
- Naranja: Importancia moderada (~0.2-0.3)
- Rojo oscuro: Importancia alta (~0.4+)

**Relaciones más fuertes (>0.4):**
- **Turbidez → Oxígeno: 0.654** (rojo oscuro)
- **Turbidez → pH: 0.529** (rojo)
- **Oxígeno → Turbidez: 0.460** (rojo)

**Relaciones moderadas (0.2-0.4):**
- Luz → Turbidez: 0.380
- Luz → Temperatura: 0.256
- pH → Temperatura: 0.344
- Oxígeno → Temperatura: 0.280
- Temperatura → Turbidez: 0.253

### Conclusiones

1. **Relación Turbidez-Oxígeno es bidireccional y más fuerte:**
   - Turbidez predice Oxígeno: 0.654 (importancia más alta de la matriz)
   - Oxígeno predice Turbidez: 0.460
   - Relación simbiótica: se afectan mutuamente

2. **Turbidez es predictor universal:**
   - Importancia alta para predecir: Oxígeno (0.654), pH (0.529), Luz (0.344)
   - Única variable con importancia >0.2 para todos los targets
   - Confirma rol central en el sistema

3. **pH tiene poder predictivo bidireccional:**
   - pH → Temperatura: 0.344 (sorprendente)
   - Turbidez → pH: 0.529
   - A pesar de baja correlación lineal, RF detecta relaciones no lineales

4. **Luz tiene rol específico:**
   - Importante para predecir turbidez (0.380) y temperatura (0.256)
   - Probablemente por ciclos día/noche que afectan temperatura del agua
   - Menos importante para pH y oxígeno directamente

5. **Temperatura es mal predictor (<0.3 para todo):**
   - Rango estrecho limita su utilidad predictiva
   - A pesar de efectos físicos conocidos, variabilidad insuficiente

6. **Diagonal vacía:** Variable no se usa para predecirse a sí misma (lógico)

**Implicación práctica:** 
- Para predecir oxígeno: usar turbidez como feature principal
- Para predecir turbidez: usar oxígeno y luz
- Para predecir pH: usar turbidez
- Temperatura y pH son predictores débiles para la mayoría de targets

---

# RESUMEN EJECUTIVO

## Principales Hallazgos

### 1. Jerarquía de Variables

**Por Importancia Predictiva (Random Forest):**
1. Turbidez (1.780) - Variable más influyente
2. Oxígeno (1.187) - Variable central
3. pH (0.813) - Importancia moderada
4. Luz (0.719) - Importancia moderada-baja
5. Temperatura (0.502) - Importancia baja

**Por Importancia Práctica (Calidad del Agua):**
1. pH - Crítico para vida acuática (87 eventos críticos detectados)
2. Oxígeno - Esencial para vida acuática
3. Turbidez - Indicador de contaminación
4. Temperatura - Afecta procesos bioquímicos
5. Luz - Influye en fotosíntesis

**Discrepancia importante:** Importancia estadística ≠ Importancia práctica

### 2. Relaciones Clave del Sistema

**Correlación más fuerte: Turbidez ↔ Oxígeno (-0.508)**
- Mecanismo: Agua turbia bloquea luz → reduce fotosíntesis → menos O₂
- Bidireccional: Bajo O₂ también puede aumentar turbidez (procesos anaeróbicos)
- Importancia RF: Turbidez predice Oxígeno con 0.654

**Segunda correlación: Temperatura ↔ Oxígeno (-0.361)**
- Mecanismo: Agua caliente retiene menos gases disueltos
- Efecto claro en boxplots: Temperatura alta → Oxígeno bajo

**Tercera correlación: Temperatura ↔ Turbidez (+0.363)**
- Mecanismo menos claro: Posiblemente actividad biológica

**pH es independiente:** Correlaciones <0.2 con todas las variables
- Controlado por factores externos no medidos
- Requiere monitoreo directo independiente

### 3. Condiciones del Sistema Monitoreado

**Temperatura:**
- Media: 21.52°C
- Rango: 16.75 - 28.94°C
- Estabilidad: Alta (CV=9.5%)
- Estado: ✓ Normal para ecosistema acuático

**pH:**
- Media: 8.06 (ligeramente alcalino)
- Rango: 7.0 - 9.6
- Distribución: Bimodal (sugiere 2 sensores con diferente calibración)
- Estado: ⚠️ 87 eventos críticos (2.7%) requieren atención

**Turbidez:**
- Media: 89.17 NTU (después de limpieza)
- 81.9% de lecturas entre 90-100 NTU
- Estado: ⚠️ Alta turbidez persistente, posible problema de contaminación

**Oxígeno:**
- Media: 8.72 mg/L
- Rango: 0.0 - 20.0 mg/L
- Variabilidad: Muy alta (CV=55.7%)
- Estado: ⚠️ Eventos con O₂ cercano a 0 mg/L (crítico)

**Luz:**
- Mediana: 10.49 Lux (baja)
- Rango extremo: 0.01 - 39,662 Lux
- Variabilidad: Extrema (CV=464.5%)
- Estado: ⚠️ Outliers sugieren mal posicionamiento del sensor ocasionalmente

### 4. Predictibilidad del Sistema

**Variables muy predecibles (R² > 0.9):**
- Turbidez (0.955) - Casi perfectamente predecible
- Oxígeno (0.926) - Muy predecible

**Variables moderadamente predecibles (0.7 < R² < 0.8):**
- Luz (0.792)
- Temperatura (0.732)
- pH (0.701)

**Conclusión:** Sistema altamente determinista, comportamiento predecible con Random Forest.

### 5. Transformaciones Aplicadas

**Luz:** log(x+1) + eliminación percentil 99
- Razón: Distribución log-normal, CV=464.5%
- Efecto: Normalización para análisis estadístico

**Turbidez:** log(x+1) + eliminación percentil 5
- Razón: Distribución sesgada, 81.9% en rango 90-100 NTU
- Efecto: Mejora visualización y reduce sesgo

---

## Recomendaciones

### Monitoreo y Alertas

1. **pH:** Sistema de alerta independiente para valores <6.5 o >9.0
   - No confiar en predicción por modelo
   - 87 eventos críticos ya detectados

2. **Oxígeno:** Alerta crítica para valores <5 mg/L
   - Esencial para vida acuática
   - Alta variabilidad requiere monitoreo continuo

3. **Turbidez:** Investigar causa de turbidez persistentemente alta
   - Posible fuente de contaminación por sedimentos
   - Afecta negativamente a oxígeno y ecosistema

### Optimización de Sensores

1. **Redundancia:** Turbidez y oxígeno parcialmente redundantes
   - Considerar reducir frecuencia de medición de una u otra
   - Mantener ambas para validación cruzada

2. **pH crítico:** No puede ser predicho confiablemente
   - Mantener medición directa frecuente
   - Considerar calibración diferencial entre sensores

3. **Luz:** Posible problema de posicionamiento
   - Valores >30,000 Lux sugieren sensor fuera del agua
   - Revisar instalación física

### Intervenciones

1. **Prioridad 1: Reducir turbidez**
   - Mayor impacto en calidad general del agua
   - Mejorará oxígeno disuelto (correlación -0.508)
   - Considerar sistemas de filtración/sedimentación

2. **Prioridad 2: Identificar causa de pH bimodal**
   - Posible diferencia de calibración entre esp32_01 y esp32_03
   - O dos condiciones ambientales diferentes
   - Requerir calibración homogénea

3. **Monitoreo de temperatura:** Aunque estable, vigilar períodos cálidos
   - Temperatura alta reduce oxígeno disponible
   - Planificar aireación en verano si es necesario

---

## Validación del Análisis

### Fortalezas

1. **Dataset robusto:** 3,011 mediciones después de limpieza (de 4,027 originales)
2. **Limpieza rigurosa:** Eliminación de valores imposibles físicamente y fallos de sensor
3. **Transformaciones justificadas:** Log-transformación apropiada para luz y turbidez
4. **Múltiples enfoques:** Correlación, boxplots, Random Forest (triangulación de resultados)
5. **Consistencia:** Resultados coherentes entre diferentes métodos de análisis

### Limitaciones

1. **Período de medición:** ~2 meses puede no capturar variaciones estacionales
2. **Variables no medidas:** Nutrientes, bacterias, otros contaminantes
3. **Outliers de luz:** Sugieren problemas ocasionales de sensor
4. **pH bimodal:** Indica posible problema de calibración no resuelto
5. **Turbidez saturada:** 81.9% en rango 90-100 NTU sugiere sensor al límite

### Próximos Pasos

1. **Análisis temporal extendido:** Evaluar estacionalidad con datos de 12 meses
2. **Análisis por dispositivo:** Comparar calibración de esp32_01 vs esp32_02 vs esp32_03
3. **Detección de eventos:** Identificar patrones específicos (lluvias, contaminación puntual)
4. **Modelo predictivo:** Implementar sistema de predicción de oxígeno basado en turbidez
5. **Dashboard en tiempo real:** Alertas automáticas para pH y oxígeno críticos

---

**Fecha de documentación:** Enero 2026  
**Proyecto:** Sensores TACHI - Monitoreo de Calidad del Agua  
**Dispositivos:** ESP32 (esp32_01, esp32_02, esp32_03)  
**Variables monitoreadas:** Temperatura, pH, Turbidez, Oxígeno Disuelto, Luz
