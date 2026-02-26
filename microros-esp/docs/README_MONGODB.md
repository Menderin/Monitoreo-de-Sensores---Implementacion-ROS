# 🗄️ Sistema de Monitoreo con MongoDB Atlas

**Última actualización:** 14 de enero de 2026

## 📊 Arquitectura

```
ESP32 → micro-ROS Agent → ROS 2 Topics → sensor_to_mongodb.py → MongoDB Atlas
  (CWT-BL)                   ↓                    ↓                  (Nube)
                        /temperatura         JSON Document
                        /ph                       ↓
                                            Archivo local (respaldo)
```

## 🚀 Uso Rápido

### 1. Configurar MongoDB Atlas

**Ver guía completa:** [README_ENV.md](README_ENV.md)

```bash
cd ~/Documentos/Github/sensores/microRostest/scripts

# Copiar plantilla de configuración
cp .env.example .env

# Editar con tus credenciales de MongoDB Atlas
nano .env
```

**Formato del archivo `.env`:**
```env
MONGO_URI=mongodb+srv://usuario:contraseña@cluster0.xxxxx.mongodb.net/?retryWrites=true&w=majority
MONGO_DB=Datos_ESP
MONGO_COLLECTION=datos_sensores
```

### 2. Instalar dependencias

```bash
# Activar entorno virtual (recomendado)
source ~/Documentos/Github/.venv/bin/activate

# Instalar paquetes
pip install python-dotenv pymongo
```

### 3. Iniciar el sistema completo

**Terminal 1 - Agente micro-ROS:**
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

**Terminal 2 - Nodo MongoDB:**
```bash
cd ~/Documentos/Github/sensores/microRostest/scripts
python3 sensor_to_mongodb.py
```

**Salida esperada:**
```
✅ Conectado a MongoDB Atlas: cluster0.xxxxx.mongodb.net
🗄️  Nodo MongoDB iniciado
📊 Base de datos: Datos_ESP.datos_sensores
====================================================================
✅ Documento guardado (ID: 67abc...) | Temp: 25.50°C | pH: 7.20
✅ Documento guardado (ID: 67abd...) | Temp: 25.48°C | pH: 7.18
```

---

## 📝 Formato de Datos JSON

### Estructura simplificada (actual)
```json
{
  "timestamp": "2026-01-14T10:30:45.123456",
  "temperatura": {
    "valor": 25.5,
    "unidad": "°C"
  },
  "ph": {
    "valor": 7.2,
    "unidad": "pH"
  }
}
```

### Ejemplo de documento en MongoDB
```json
{
  "_id": ObjectId("69679493e0ea50bc06a02ad0"),
  "timestamp": "2026-01-14T10:30:45.123456",
  "temperatura": {
    "valor": 25.5,
    "unidad": "°C"
  },
  "ph": {
    "valor": 7.2,
    "unidad": "pH"
  }
}
```

**Ventajas del diseño simplificado:**
- ✅ Sin redundancia de timestamps
- ✅ Estructura limpia y directa
- ✅ Fácil de consultar y analizar
- ✅ Menor tamaño de documentos

---

## 🗄️ Configuración MongoDB Atlas (Cloud)

### Paso 1: Crear cuenta y cluster

1. Ve a https://www.mongodb.com/cloud/atlas
2. Crea una cuenta gratuita
3. Crea un cluster gratuito (M0 Sandbox)
4. Espera a que se despliegue (~5 minutos)

### Paso 2: Configurar acceso

**Database Access (usuarios):**
1. Ve a **Database Access**
2. Click **"Add New Database User"**
3. Crea usuario con permisos **Read and write to any database**
4. Guarda usuario y contraseña

**Network Access (IPs):**
1. Ve a **Network Access**
2. Click **"Add IP Address"**
3. Opción A: **Add Current IP Address** (más seguro)
4. Opción B: **Allow Access from Anywhere** (`0.0.0.0/0`) - para desarrollo

### Paso 3: Obtener string de conexión

1. Ve a tu cluster → **"Connect"**
2. Selecciona **"Connect your application"**
3. Driver: **Python**, Version: **3.12 or later**
4. Copia el connection string:
   ```
   mongodb+srv://usuario:<password>@cluster0.xxxxx.mongodb.net/?retryWrites=true&w=majority
   ```
5. **Reemplaza `<password>`** con tu contraseña real

### Paso 4: Configurar archivo .env

```bash
cd ~/Documentos/Github/sensores/microRostest/scripts
nano .env
```

**Contenido del archivo `.env`:**
```env
# Configuración MongoDB Atlas
MONGO_URI=mongodb+srv://usuario:tu_contraseña_real@cluster0.xxxxx.mongodb.net/?retryWrites=true&w=majority
MONGO_DB=Datos_ESP
MONGO_COLLECTION=datos_sensores
```

⚠️ **IMPORTANTE:** 
- NO incluyas los símbolos `<` ni `>` alrededor de la contraseña
- Elimina cualquier espacio antes/después de la contraseña
- El archivo `.env` está en `.gitignore` (no se sube a Git)
- Si tu contraseña tiene caracteres especiales, URL-encódelos:
  - `@` → `%40`
  - `#` → `%23`
  - `/` → `%2F`

---

## 📂 Estructura de Archivos

```
scripts/
├── sensor_to_mongodb.py      # Nodo principal de guardado
├── .env                       # Credenciales (NO subir a Git) ❌
├── .env.example              # Plantilla de configuración ✅
├── datos_sensores/           # Respaldo local (JSON Lines)
│   ├── sensores_2026-01-14.jsonl
│   └── sensores_2026-01-15.jsonl
├── README_ENV.md             # Guía de configuración
└── README_MONGODB.md         # Este archivo
```

---

## 🔍 Verificar Datos en MongoDB Compass

### Instalar MongoDB Compass

```bash
# Descargar desde: https://www.mongodb.com/try/download/compass
# O instalar con snap:
sudo snap install mongodb-compass
```

### Conectar

1. Abre MongoDB Compass
2. Pega tu **MONGO_URI** (el mismo del archivo `.env`)
3. Click **"Connect"**
4. Navega a: `Datos_ESP` → `datos_sensores`
5. Verás los documentos guardados en tiempo real

---

## 📊 Consultas Útiles en MongoDB

### Desde MongoDB Compass (interfaz gráfica)

**Ver últimos 10 documentos:**
```javascript
// Pestaña "Filter" (vacío)
// Pestaña "Sort": { "timestamp": -1 }
// Limit: 10
```

**Filtrar por rango de pH:**
```javascript
{ "ph.valor": { $gte: 7.0, $lte: 7.5 } }
```

**Filtrar por temperatura:**
```javascript
{ "temperatura.valor": { $gt: 25 } }
```

### Desde código Python

```python
from pymongo import MongoClient
import os
from dotenv import load_dotenv

load_dotenv()
client = MongoClient(os.getenv('MONGO_URI'))
db = client['Datos_ESP']
collection = db['datos_sensores']

# Últimos 10 documentos
docs = collection.find().sort('timestamp', -1).limit(10)
for doc in docs:
    print(doc)

# Promedio de pH
pipeline = [
    {"$group": {
        "_id": None,
        "avg_ph": {"$avg": "$ph.valor"}
    }}
]
result = list(collection.aggregate(pipeline))
print(f"pH promedio: {result[0]['avg_ph']}")
```

---

## 🛠️ Troubleshooting

### Error: "bad auth : authentication failed"
- ✅ Verifica que la contraseña en `.env` es correcta
- ✅ No incluyas `<` ni `>`
- ✅ Verifica que el usuario existe en Database Access

### Error: "MONGO_URI no configurado"
- ✅ Asegúrate de tener el archivo `.env` en `scripts/`
- ✅ Verifica que no tiene errores de sintaxis
- ✅ Reinicia el script después de editar `.env`

### Error: "python-dotenv could not parse"
- ✅ No uses comillas extra en `.env`
- ✅ Formato correcto: `CLAVE=valor` (sin espacios alrededor del `=`)

### No se guardan documentos
- ✅ Verifica que el micro-ROS Agent está corriendo
- ✅ Verifica que el ESP32 está publicando datos: `ros2 topic echo /temperatura`
- ✅ Revisa los logs del script para errores de conexión

---

## 🎓 Próximos Pasos

- [ ] Implementar `esp_id` para identificar múltiples ESP32
- [ ] Crear colección `info_esp` con metadatos de dispositivos
- [ ] Dashboard web con Grafana + MongoDB
- [ ] Alertas automáticas por valores fuera de rango
- [ ] Exportar datos a CSV para análisis

---

## 📚 Recursos Adicionales

- [MongoDB Atlas Documentation](https://docs.atlas.mongodb.com/)
- [PyMongo Tutorial](https://pymongo.readthedocs.io/)
- [MongoDB Compass Guide](https://docs.mongodb.com/compass/)
- [README_ENV.md](README_ENV.md) - Configuración detallada de variables de entorno
