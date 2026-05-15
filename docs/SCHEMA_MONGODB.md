# Esquema de MongoDB (Monitoreo de Sensores)

Este documento describe la estructura real de las colecciones usadas por el proyecto para almacenar lecturas (`sensors_data`) y metadatos de dispositivos (`devices_data`).

> Importante: este documento **no** incluye credenciales. La conexión se configura vía `database/.env`.

---

## Configuración (.env)

Variables típicas:

```env
MONGO_URI=...
MONGO_DB=Datos_ESP
MONGO_COLLECTION=sensors_data
MONGO_COLLECTION_DISPOSITIVOS=devices_data
```

---

## Colección: `sensors_data` (Time Series)

### Propósito
Guardar lecturas periódicas por dispositivo. El flujo principal es:

- Firmware publica `/sensor_data` (ROS 2)
- `database/ros_sensor_node.py` consume el topic
- `database/modules/service.py` inserta documentos en MongoDB

### Time Series (creación)
La colección se crea como *Time Series* con:

- `timeField`: `timestamp`
- `metaField`: `dispositivo_id`
- `granularity`: `minutes`

### Documento (estructura)

Campos:

- `timestamp` (**Date**, UTC): momento de la lectura.
- `dispositivo_id` (**string**): ID del dispositivo en formato MAC sin separadores, ej. `"AABBCCDDEEFF"`.
- `datos` (**object**): valores medidos. Puede venir parcial.
  - `datos.temperatura` (**number**): temperatura en °C (ya redondeada a 2 decimales).
  - `datos.ph` (**number**): pH (ya redondeado a 2 decimales).
- `telemetria` (**object**, opcional): diccionario libre con datos adicionales (si el nodo los provee).

### Ejemplo
> Nota: en MongoDB el `timestamp` es tipo Date (BSON). En JSON “de ejemplo” se suele escribir como `ISODate(...)`.

```json
{
  "timestamp": "ISODate(2026-05-06T14:05:12.123Z)",
  "dispositivo_id": "AABBCCDDEEFF",
  "datos": {
    "temperatura": 25.34,
    "ph": 7.12
  },
  "telemetria": {
    "rssi": -62,
    "heap_free": 123456
  }
}
```

### Índices
Índice compuesto recomendado (y creado en migraciones):

- `{ dispositivo_id: 1, timestamp: -1 }`

---

## Colección: `devices_data` (metadatos)

### Propósito
Un documento por dispositivo con:

- estado operativo
- configuración
- calibración (offsets)
- estadísticas de conexión

### Documento (estructura “completa”)

Clave primaria:

- `_id` (**string**): mismo valor que `dispositivo_id` (MAC sin separadores).

Campos típicos:

- `nombre` (**string**): nombre descriptivo.
- `alias` (**string**, opcional): nombre “humano” usado en reportes (si no existe, se usa `_id`).
- `estado` (**string**): `"pendiente" | "activo" | "mantenimiento" | "inactivo"`.
- `auto_registrado` (**bool**): si fue detectado automáticamente.
- `ubicacion` (**string | null**).
- `configuracion` (**object**):
  - `firmware_version` (**string**)
  - `intervalo_lectura_seg` (**number**)
  - `sensores_habilitados` (**array<string>**), ej. `["ph", "temperatura"]`
- `calibracion` (**object**): offsets aplicados al guardar lecturas
  - `ph_offset` (**number**)
  - `temp_offset` (**number**)
- `unidades` (**object**):
  - `temperatura` (**string**), ej. `"°C"`
  - `ph` (**string**), ej. `"pH"`
- `conexion` (**object**):
  - `primera` (**Date**)
  - `ultima` (**Date**)
  - `total_lecturas` (**int**)

### Ejemplo

```json
{
  "_id": "AABBCCDDEEFF",
  "nombre": "Dispositivo EEFF",
  "alias": "Tanque 1",
  "estado": "pendiente",
  "auto_registrado": true,
  "ubicacion": null,
  "configuracion": {
    "firmware_version": "desconocido",
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
    "primera": "ISODate(2026-05-06T13:00:00.000Z)",
    "ultima": "ISODate(2026-05-06T14:05:12.123Z)",
    "total_lecturas": 42
  }
}
```

### Variante “mínima” (auto-registro desde scripts)
Algunos scripts (por ejemplo la herramienta de calibración de pH) pueden insertar inicialmente un documento reducido con:

- `_id`, `nombre`, `auto_registrado`, `ubicacion`, `conexion.{primera,ultima,total_lecturas}`

Luego el flujo principal puede completar campos como `estado`, `configuracion`, `calibracion` y `unidades`.

### Índices
Índice recomendado (y creado en migraciones):

- `{ estado: 1 }`

---

## Consultas típicas (MongoDB Compass / shell)

### Últimas lecturas de un dispositivo

Filtro:

```javascript
{ "dispositivo_id": "AABBCCDDEEFF" }
```

Sort:

```javascript
{ "timestamp": -1 }
```

### Lecturas por ventana de tiempo (UTC)

```javascript
{
  "dispositivo_id": "AABBCCDDEEFF",
  "timestamp": {
    "$gte": ISODate("2026-05-06T00:00:00Z"),
    "$lte": ISODate("2026-05-06T12:00:00Z")
  }
}
```

### Buscar dispositivos activos

```javascript
{ "estado": "activo" }
```

---

## Notas importantes

- `timestamp` y `conexion.{primera,ultima}` deben ser tipo **Date** en MongoDB para consultas por rango eficientes.
- `datos.temperatura` y `datos.ph` se guardan ya **calibrados** por offsets (`devices_data.calibracion`).
- El formato de ID de dispositivo usado en el sistema es MAC **sin separadores** (ej. `AABBCCDDEEFF`).
