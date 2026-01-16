# Dashboard de Monitoreo Ambiental - Estructura Modular

## 📁 Estructura del Proyecto

```
main/
├── app.py                      # Punto de entrada principal (~60 líneas)
├── app_original.py             # Backup de la versión monolítica
├── requirements.txt            # Dependencias Python
├── .env                        # Variables de entorno
│
├── config/                     # ⚙️ Configuración
│   ├── __init__.py
│   ├── settings.py            # Configuración centralizada
│   └── colors.py              # Paleta de colores
│
├── styles/                     # 🎨 Estilos CSS
│   ├── __init__.py
│   └── custom_css.py          # Estilos Streamlit personalizados
│
├── database/                   # 💾 Gestión de datos
│   ├── __init__.py
│   └── mongo_handler.py       # Conexión y consultas MongoDB
│
├── components/                 # 🧩 Componentes reutilizables
│   ├── __init__.py
│   ├── sidebar.py             # Sidebar con controles
│   └── charts.py              # Gráficos Plotly reutilizables
│
├── pages/                      # 📄 Páginas del dashboard
│   ├── __init__.py
│   ├── monitoreo_vivo.py      # Vista de monitoreo en tiempo real
│   ├── analisis_estadistico.py # Vista de análisis estadístico
│   ├── registros.py           # Vista de tabla de registros
│   └── dispositivos.py        # Vista de información de dispositivos
│
├── utils/                      # 🛠️ Utilidades
│   ├── __init__.py
│   └── helpers.py             # Funciones auxiliares
│
└── assets/                     # 🖼️ Recursos multimedia
    ├── microalgas.webp
    └── ...
```

## 🚀 Ventajas de la Arquitectura Modular

### ✅ Mantenibilidad
- Código organizado por responsabilidades
- Fácil localización de bugs
- Cambios localizados sin afectar otros módulos

### ✅ Escalabilidad
- Agregar nuevos sensores: crear módulo en `pages/`
- Nuevos gráficos: añadir función en `components/charts.py`
- Nuevas fuentes de datos: extender `database/`

### ✅ Reutilización
- Componentes compartidos entre páginas
- Funciones de gráficos parametrizables
- Configuración centralizada

### ✅ Testabilidad
- Módulos independientes fáciles de testear
- Mocking simplificado de base de datos
- Testing unitario por componente

### ✅ Colaboración
- Múltiples desarrolladores trabajando en paralelo
- Menos conflictos de merge en Git
- Revisión de código más sencilla

## 📦 Instalación

```bash
cd main
pip install -r requirements.txt
```

## ▶️ Ejecución

```bash
streamlit run app.py
```

## 🔧 Configuración

Edita el archivo `.env`:

```env
MONGO_URI=mongodb://localhost:27017/
MONGO_DB=sensores_db
MONGO_COLLECTION=lecturas
```

## 📝 Agregar Nueva Página

1. Crear archivo en `pages/nueva_pagina.py`:

```python
import streamlit as st

def render_nueva_pagina(df):
    st.markdown("### Mi Nueva Página")
    # Tu código aquí
```

2. Importar en `pages/__init__.py`:

```python
from .nueva_pagina import render_nueva_pagina
__all__ = [..., 'render_nueva_pagina']
```

3. Agregar tab en `app.py`:

```python
tab5 = st.tabs([..., "MI NUEVA PÁGINA"])
with tab5:
    render_nueva_pagina(df)
```

## 🎨 Agregar Nuevo Gráfico

En `components/charts.py`:

```python
def crear_nuevo_grafico(df, columna):
    fig = go.Figure()
    # Configurar gráfico
    return fig
```

## 🔄 Migración desde Versión Original

La versión original está respaldada en `app_original.py`.

**Diferencias clave:**
- Antes: 729 líneas en un archivo
- Ahora: ~60 líneas en `app.py` + módulos especializados
- Funcionalidad: 100% idéntica
- Performance: Mejorado con mejor cache

## 📊 Comparación

| Aspecto | Versión Original | Versión Modular |
|---------|------------------|-----------------|
| Archivos | 1 | 17 |
| Líneas en main | 729 | ~60 |
| Mantenibilidad | ⭐⭐ | ⭐⭐⭐⭐⭐ |
| Escalabilidad | ⭐⭐ | ⭐⭐⭐⭐⭐ |
| Testabilidad | ⭐ | ⭐⭐⭐⭐⭐ |
| Reutilización | ⭐ | ⭐⭐⭐⭐⭐ |

## 🐛 Debugging

Cada módulo tiene logging independiente. Para debug:

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

## 📚 Documentación de Módulos

- **config/settings.py**: Todas las constantes de configuración
- **database/mongo_handler.py**: Gestión de MongoDB con cache
- **components/sidebar.py**: Controles del panel lateral
- **components/charts.py**: Biblioteca de gráficos Plotly
- **pages/*.py**: Vistas independientes del dashboard

## 🤝 Contribuir

1. Fork del proyecto
2. Crear branch: `git checkout -b feature/nueva-funcionalidad`
3. Commit cambios: `git commit -m 'Agregar nueva funcionalidad'`
4. Push: `git push origin feature/nueva-funcionalidad`
5. Pull Request

## 📄 Licencia

Ver archivo LICENSE en el repositorio principal.
