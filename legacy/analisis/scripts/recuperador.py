import pandas as pd
import json

# 1. Cargar el archivo 'gigante' (pero ligero en peso)
input_file = '../data/Datos_ESP.datos_sensoresV2.json'  # Pon aquí el nombre de tu archivo original
output_file = '../data/data_limpia_para_mongo.json'

print("Cargando datos...")
with open(input_file, 'r', encoding='utf-8') as f:
    data = json.load(f)

# Convertimos a DataFrame de Pandas
df = pd.DataFrame(data)

# 2. Extraer el valor de temperatura del diccionario anidado
df['temp_valor'] = df['temperatura'].apply(lambda x: x['valor'] if isinstance(x, dict) else x)

# 3. Convertir timestamp a objeto datetime para poder manipularlo
# Esto no modifica el string original, solo crea una representación interna
df['temp_time'] = pd.to_datetime(df['timestamp'])

# 4. Redondear al segundo para encontrar colisiones
# 'floor' baja al segundo exacto (ignora los milisegundos para la comparación)
df['segundo_exacto'] = df['temp_time'].dt.floor('s')

# Contar registros antes
total_antes = len(df)

# 5. ELIMINAR DUPLICADOS 
df_limpio = df.drop_duplicates(subset=['segundo_exacto'], keep='last')

# Contar después de eliminar duplicados
total_sin_duplicados = len(df_limpio)
duplicados_eliminados = total_antes - total_sin_duplicados

# 6. ELIMINAR REGISTROS CON TEMPERATURA NEGATIVA
# (sensor desconectado)
df_limpio = df_limpio[df_limpio['temp_valor'] >= 0]

# Contar registros después
total_despues = len(df_limpio)
negativos_eliminados = total_sin_duplicados - total_despues

print(f"Registros iniciales:       {total_antes}")
print(f"Duplicados eliminados:     {duplicados_eliminados}")
print(f"Temp. negativas eliminadas: {negativos_eliminados}")
print(f"Registros finales:         {total_despues}")

# 7. Limpieza final antes de guardar
# Eliminamos las columnas auxiliares que creamos para el cálculo
df_limpio = df_limpio.drop(columns=['temp_valor', 'temp_time', 'segundo_exacto'])

# 8. Guardar como JSON
# orient='records' crea la estructura de lista: [{...}, {...}]
print("Guardando archivo limpio...")
df_limpio.to_json(output_file, orient='records', indent=2, date_format='iso')

print("¡Listo! Ya puedes subir el archivo limpio a MongoDB.")