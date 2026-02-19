# 📊 Guía de Instalación y Configuración del Dashboard

Esta guía te ayudará a configurar y levantar el dashboard de monitoreo en tiempo real de sensores.

---

## 📋 Requisitos Previos

- **Sistema Operativo:** Ubuntu 22.04 LTS (o compatible)
- **Python:** 3.12+
- **MongoDB:** Cuenta en MongoDB Atlas (o instancia local)
- **Conda:** Para gestión de entornos virtuales (recomendado)

---

## 🚀 Instalación Paso a Paso

### 1️⃣ Clonar el Repositorio

```bash
git clone https://github.com/Menderin/sensores.git
cd sensores
```

### 2️⃣ Crear Entorno Virtual con Conda

```bash
# Crear entorno con Python 3.12
conda create -n sensores python=3.12

# Activar entorno
conda activate sensores
```

> **Alternativa con venv:**
> ```bash
> python3.12 -m venv venv
> source venv/bin/activate
> ```

### 3️⃣ Instalar Dependencias

```bash
# Instalar paquetes requeridos
pip install -r main/requirements.txt
```

**Paquetes principales instalados:**
- `streamlit` - Framework del dashboard
- `pymongo` - Cliente de MongoDB
- `pandas` - Manipulación de datos
- `plotly` - Gráficos interactivos
- `python-dotenv` - Carga de variables de entorno

### 4️⃣ Configurar MongoDB

#### Opción A: MongoDB Atlas (Nube - Recomendado)

1. **Crear cuenta gratuita** en [MongoDB Atlas](https://www.mongodb.com/cloud/atlas/register)

2. **Crear un cluster:**
   - Selecciona el plan FREE (M0)
   - Elige la región más cercana
   - Nombra tu cluster (ej: `sensores-cluster`)

3. **Configurar acceso:**
   - Database Access → Add New Database User
   - Crea usuario y contraseña
   - Network Access → Add IP Address → Allow Access from Anywhere (0.0.0.0/0)

4. **Obtener URI de conexión:**
   - Clusters → Connect → Connect your application
   - Copia el URI (formato: `mongodb+srv://usuario:password@cluster.mongodb.net/`)

#### Opción B: MongoDB Local

```bash
# Instalar MongoDB Community Edition
sudo apt-get update
sudo apt-get install -y mongodb-org

# Iniciar servicio
sudo systemctl start mongod
sudo systemctl enable mongod
```

### 5️⃣ Configurar Variables de Entorno

Crea el archivo `.env` en la carpeta `database/`:

```bash
cd database
nano .env
```

**Contenido del archivo `.env`:**

```bash
# MongoDB Atlas (Nube)
MONGO_URI=mongodb+srv://usuario:password@cluster.mongodb.net/

# O MongoDB Local
# MONGO_URI=mongodb://localhost:27017/

# Nombre de la base de datos
MONGO_DB=sensor_db

# Nombre de la colección de datos
MONGO_COLLECTION=datos_sensores

# Nombre de la colección de dispositivos
MONGO_COLLECTION_DISPOSITIVOS=dispositivos
```

> **⚠️ IMPORTANTE:** 
> - Reemplaza `usuario` y `password` con tus credenciales reales
> - No uses comillas en los valores
> - No compartas este archivo (ya está en `.gitignore`)

**Ejemplo real:**
```bash
MONGO_URI=mongodb+srv://admin:MySecurePass123@sensores.abc123.mongodb.net/
MONGO_DB=sensor_db
MONGO_COLLECTION=datos_sensores
MONGO_COLLECTION_DISPOSITIVOS=dispositivos
```

### 6️⃣ Verificar Estructura de Directorios

Asegúrate de que tienes la siguiente estructura:

```
sensores/
├── database/
│   └── .env                    # ← Tu archivo de configuración
├── main/
│   ├── app.py                  # ← Aplicación principal del dashboard
│   ├── requirements.txt
│   ├── components/             # Componentes reutilizables
│   ├── database/              # Handler de MongoDB
│   ├── pages/                 # Páginas del dashboard
│   └── styles/                # Estilos CSS personalizados
└── README.md
```

---

## ▶️ Levantar el Dashboard

### Iniciar el Dashboard

```bash
# Desde la raíz del proyecto
cd main

# Activar entorno (si no está activo)
conda activate sensores

# Ejecutar dashboard
streamlit run app.py
```

### Acceder al Dashboard

El dashboard se abrirá automáticamente en tu navegador en:

- **URL Local:** http://localhost:8501
- **URL de Red:** http://192.168.X.X:8501 (para acceso desde otros dispositivos)

---

## 📱 Páginas del Dashboard

El dashboard tiene 4 pestañas principales:

1. **🏠 INICIO** - Información general del proyecto
2. **📊 MONITOREO EN VIVO** - Datos en tiempo real con auto-refresh (10s)
3. **📈 ANÁLISIS ESTADÍSTICO** - Estadísticas y distribuciones
4. **📂 REGISTROS** - Tabla de datos históricos

---

## ⚙️ Configuración del Dashboard

### Ajustar Intervalo de Auto-Refresh

Edita el archivo `main/config/settings.py`:

```python
# Tiempo de caché en segundos (también controla el refresh)
CACHE_TTL = 10  # Cambiar a tu valor deseado

# Intervalo de auto-refresh en segundos
AUTO_REFRESH_INTERVAL = 10  # Cambiar a tu valor deseado
```

### Personalizar Rango de Horas en Sidebar

En la sidebar del dashboard puedes:
- Seleccionar rango de horas (1, 6, 12, 24 horas)
- Ver información del sistema
- Acceder a documentación

---

## 🔍 Verificación de Funcionamiento

### 1. Verificar Conexión a MongoDB

Ejecuta el siguiente script Python:

```python
from pymongo import MongoClient
from dotenv import load_dotenv
import os

load_dotenv('database/.env')

uri = os.getenv('MONGO_URI')
db_name = os.getenv('MONGO_DB')

client = MongoClient(uri)
db = client[db_name]

print(f"✅ Conectado a MongoDB: {db_name}")
print(f"📦 Colecciones: {db.list_collection_names()}")
```

### 2. Verificar Datos en MongoDB

Ve a MongoDB Atlas:
- Collections → Browse Collections
- Selecciona `sensor_db` → `datos_sensores`
- Deberías ver documentos con esta estructura:

```json
{
  "_id": "...",
  "timestamp": "2026-01-21T19:30:00Z",
  "dispositivo_id": "A1:B2:C3:D4:E5:F6",
  "datos": {
    "temperatura": 23.5,
    "ph": 7.4
  }
}
```

---

## 🛠️ Troubleshooting

### Error: "No module named 'streamlit'"

```bash
# Asegúrate de estar en el entorno correcto
conda activate sensores

# Reinstala dependencias
pip install -r main/requirements.txt
```

### Error: "Can't connect to MongoDB"

1. **Verifica el URI:**
   - Asegúrate de que el URI en `.env` es correcto
   - No debe tener comillas
   - Debe incluir usuario y password

2. **Verifica Network Access en MongoDB Atlas:**
   - Debe estar habilitado `0.0.0.0/0` (o tu IP específica)

3. **Prueba la conexión:**
   ```bash
   python -c "from pymongo import MongoClient; print(MongoClient('TU_URI').list_database_names())"
   ```

### Warning: "use_container_width is deprecated"

Este es un warning de deprecación de Streamlit. No afecta la funcionalidad, pero puedes ignorarlo por ahora o actualizar a la nueva sintaxis cuando sea necesario.

### Dashboard se ve en blanco o sin datos

1. **Verifica que hay datos en MongoDB:**
   - Revisa MongoDB Atlas o local
   - Asegúrate de que el nodo ROS está publicando datos

2. **Verifica el rango de horas:**
   - En la sidebar, intenta cambiar el rango a "24 horas"
   - Si hay datos más antiguos, deberían aparecer

3. **Revisa los logs:**
   - Mira la terminal donde ejecutaste `streamlit run app.py`
   - Busca errores o mensajes de conexión

---

## 🎨 Características del Dashboard

- ✅ **Auto-refresh cada 10 segundos** - Sin recargar la página completa
- ✅ **Métricas en tiempo real** - pH y Temperatura actuales y promedio
- ✅ **Deltas con indicadores** - Flechas ↑↓ que muestran tendencias
- ✅ **Gráficos interactivos** - Series temporales y box plots
- ✅ **Ejes dinámicos** - Rango Y se ajusta automáticamente a los datos
- ✅ **Timezone correcto** - Muestra hora de Chile (UTC-3)
- ✅ **Diseño responsive** - Se adapta a diferentes tamaños de pantalla
- ✅ **Estilos personalizados** - Tema oscuro con acentos en cyan/verde

---

## 📦 Actualizar el Dashboard

```bash
# Actualizar código desde GitHub
cd sensores
git pull origin main

# Actualizar dependencias (si hubo cambios)
conda activate sensores
pip install -r main/requirements.txt --upgrade

# Reiniciar dashboard
cd main
streamlit run app.py
```

---

## 🔐 Seguridad

> **⚠️ NUNCA compartas tu archivo `.env`**

- El archivo `.env` contiene credenciales sensibles
- Ya está incluido en `.gitignore`
- No lo subas a GitHub ni lo compartas públicamente
- Usa variables de entorno o secretos en producción

---

## 📞 Soporte

Si tienes problemas:

1. Revisa la sección [Troubleshooting](#-troubleshooting)
2. Verifica los logs en la terminal
3. Abre un [Issue en GitHub](https://github.com/Menderin/sensores/issues)

---

**✨ ¡Listo! Tu dashboard debería estar funcionando.**
