# Configuración de Variables de Entorno

## 📋 Pasos para configurar MongoDB Atlas

### 1. Instalar dependencias
```bash
pip install python-dotenv pymongo
```

### 2. Crear archivo .env
```bash
cd /home/lab-ros/Documentos/Github/sensores/microRostest/scripts
cp .env.example .env
```

### 3. Editar .env con tus credenciales

Abre el archivo `.env` y completa con tu información real:

```bash
# Obtén este string desde MongoDB Atlas:
# Cluster → Connect → Connect your application → Copy connection string
MONGO_URI=mongodb+srv://usuario:contraseña@cluster0.xxxxx.mongodb.net/?retryWrites=true&w=majority

# Nombre de tu base de datos
MONGO_DB=Datos_ESP

# Nombre de tu colección
MONGO_COLLECTION=datos_sensores
```

### 4. Obtener el string de conexión desde MongoDB Atlas

1. Ve a [MongoDB Atlas](https://cloud.mongodb.com/)
2. Entra a tu cluster
3. Click en **"Connect"**
4. Selecciona **"Connect your application"**
5. Copia el string de conexión
6. **Reemplaza** `<password>` con tu contraseña real
7. Pégalo en `MONGO_URI` del archivo `.env`

### 5. Verificar que .env está en .gitignore

El archivo `.env` **NO debe subirse a Git**. Verifica que esté en `.gitignore`:

```bash
cat ../.gitignore | grep .env
```

Deberías ver:
```
.env
.env.local
```

## ✅ Usar el nodo

```bash
python3 sensor_to_mongodb.py
```

## 🔒 Seguridad

- ✅ `.env` está en `.gitignore` (credenciales seguras)
- ✅ `.env.example` es la plantilla (sí se sube a Git)
- ✅ El código oculta la contraseña en los logs
- ❌ **NUNCA** subas `.env` a Git
