import streamlit as st
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from pymongo import MongoClient
from datetime import datetime, timedelta
from dotenv import load_dotenv
import os
import base64
from pathlib import Path

# Cargar variables de entorno
load_dotenv()

# Función para cargar imagen local
def get_image_base64(image_path):
    """Convierte imagen a base64 para uso en HTML"""
    try:
        with open(image_path, "rb") as image_file:
            image_bytes = image_file.read()
        return base64.b64encode(image_bytes).decode()
    except:
        return None

# Configuración de la página
st.set_page_config(
    page_title="Sistema de Monitoreo Ambiental",
    page_icon="🌊",
    layout="wide",
    initial_sidebar_state="expanded"
)



# CSS personalizado con animaciones y fondo vivo
st.markdown("""
    <style>
    /* Quitar padding y margen global para pegar todo arriba */
    html, body, .stApp {
        padding: 0 !important;
        margin: 0 !important;
    }

    /* Contenedor principal sin padding adicional */
    div[data-testid="stAppViewContainer"] > .main {
        padding-top: 0 !important;
    }

    /* Ocultar/header para evitar espacio superior */
    header[data-testid="stHeader"] {
        height: 0px;
        min-height: 0px;
        visibility: hidden;
    }

    /* Eliminar padding superior del contenido principal */
    .main .block-container {
        padding-top: 0 !important;
        padding-bottom: 2rem;
        margin-top: 0 !important;
    }
    
    /* Fondo simple oscuro sin gradiente animado */
    .stApp {
        background: #0f2027 !important;
    }
    
    /* Estilo de tarjetas */
    div[data-testid="metric-container"] {
        background: rgba(255, 255, 255, 0.1);
        backdrop-filter: blur(10px);
        border: 1px solid rgba(255, 255, 255, 0.2);
        border-radius: 15px;
        padding: 15px;
        box-shadow: 0 8px 32px 0 rgba(31, 38, 135, 0.37);
    }
    
    /* Texto de métricas */
    div[data-testid="metric-container"] label {
        color: #00D9FF !important;
        font-weight: 600;
    }
    
    div[data-testid="metric-container"] [data-testid="stMetricValue"] {
        color: #FFFFFF !important;
        font-size: 2rem;
    }
    
    /* Títulos */
    h1, h2, h3 {
        color: #FFFFFF !important;
        text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
    }
    
    /* Sidebar */
    section[data-testid="stSidebar"] {
        background: rgba(15, 32, 39, 0.95);
        backdrop-filter: blur(10px);
    }
    
    section[data-testid="stSidebar"] * {
        color: #FFFFFF !important;
    }
    
    /* Tablas */
    .stDataFrame {
        background: rgba(255, 255, 255, 0.1);
        backdrop-filter: blur(10px);
        border-radius: 10px;
    }
    </style>
""", unsafe_allow_html=True)

# Paleta de colores vivos
COLORS = {
    'ph': '#00D9FF',           # Cyan brillante
    'temperatura': '#FF6B35',   # Naranja vibrante
    'success': '#4ADE80',       # Verde neón
    'warning': '#FBBF24',       # Amarillo brillante
    'danger': '#EF4444',        # Rojo vibrante
    'primary': '#8B5CF6',       # Púrpura
    'secondary': '#EC4899'      # Rosa
}

# Conexión a MongoDB
@st.cache_resource
def get_mongo_client():
    """Conecta a MongoDB y retorna el cliente"""
    try:
        client = MongoClient(os.getenv('MONGO_URI'))
        return client
    except Exception as e:
        st.error(f"Error al conectar con MongoDB: {e}")
        return None

# Cargar datos de MongoDB
@st.cache_data(ttl=60)  # Cache de 60 segundos
def cargar_datos_mongo(horas=1):
    """Carga datos de MongoDB de las últimas N horas"""
    try:
        client = get_mongo_client()
        if client is None:
            return pd.DataFrame()
        
        db = client[os.getenv('MONGO_DB')]
        collection = db[os.getenv('MONGO_COLLECTION')]
        
        # Calcular tiempo límite
        tiempo_limite = datetime.now() - timedelta(hours=horas)
        
        # Consulta a MongoDB
        query = {
            'timestamp': {'$gte': tiempo_limite.isoformat()}
        }
        
        datos = list(collection.find(query).sort('timestamp', 1))
        
        if not datos:
            return pd.DataFrame()
        
        # Procesar datos
        registros = []
        for registro in datos:
            try:
                registros.append({
                    'timestamp': pd.to_datetime(registro.get('timestamp')),
                    'ph': registro.get('ph', {}).get('valor'),
                    'temperatura': registro.get('temperatura', {}).get('valor')
                })
            except:
                continue
        
        df = pd.DataFrame(registros)
        df = df.sort_values('timestamp').reset_index(drop=True)
        
        return df
    except Exception as e:
        st.error(f"Error al cargar datos: {e}")
        return pd.DataFrame()

# Sidebar
st.sidebar.markdown("### Panel de Control")

# Selector de rango temporal
rango_horas = st.sidebar.selectbox(
    "Rango Temporal",
    options=[1, 3, 6, 12, 24],
    index=0,
    format_func=lambda x: f"Última{'s' if x > 1 else ''} {x} hora{'s' if x > 1 else ''}"
)

# Botón de actualización
if st.sidebar.button(" Actualizar Datos", type="primary"):
    st.cache_data.clear()
    st.rerun()

# Auto-refresh
auto_refresh = st.sidebar.checkbox("Auto-actualización (60s)", value=False)
if auto_refresh:
    st.sidebar.info("Actualizando automáticamente cada 60 segundos...")

# Cargar datos
df = cargar_datos_mongo(horas=rango_horas)

if not df.empty:
    # Información de conexión
    st.sidebar.success(f"Conectado a MongoDB")
    st.sidebar.metric("Registros Cargados", len(df))
    st.sidebar.metric("Último Registro", 
                     df['timestamp'].max().strftime("%H:%M:%S"))
    
    # Navegación con pestañas estilo barra superior
    st.markdown("""
        <style>
        /* Estilo de pestañas como barra de navegación pegada arriba */
        .stTabs {
            margin-top: 0 !important;
        }
        .stTabs [data-baseweb="tab-list"] {
            gap: 8px;
            background: rgba(255, 255, 255, 0.05);
            padding: 8px 20px;
            border-radius: 8px;
            margin-top: 0 !important;
            margin-bottom: 10px;
        }
        
        .stTabs [data-baseweb="tab"] {
            background: transparent;
            color: #FFFFFF;
            font-weight: 600;
            font-size: 0.95rem;
            padding: 12px 30px;
            border: none;
            border-radius: 5px;
            transition: all 0.3s ease;
        }
        
        .stTabs [data-baseweb="tab"]:hover {
            background: rgba(0, 217, 255, 0.2);
            color: #00D9FF;
        }
        
        .stTabs [aria-selected="true"] {
            background: linear-gradient(90deg, #00D9FF, #8B5CF6) !important;
            color: #FFFFFF !important;
            box-shadow: 0 4px 15px rgba(0, 217, 255, 0.4);
        }
        
        /* Header debajo de las pestañas */
        .main-header {
            text-align: center;
            padding: 15px 20px;
            background: rgba(0, 217, 255, 0.05);
            border-radius: 15px;
            margin-bottom: 15px;
            border: 1px solid rgba(0, 217, 255, 0.2);
        }
        
        .main-header h1 {
            font-size: 2rem;
            margin-bottom: 5px;
        }
        
        .main-header p {
            font-size: 1rem;
            color: #00D9FF;
            margin: 0;
        }
        </style>
    """, unsafe_allow_html=True)
    
    # Crear pestañas de navegación
    tab1, tab2, tab3, tab4 = st.tabs([
        "MONITOREO EN VIVO",
        "ANÁLISIS ESTADÍSTICO", 
        "REGISTROS",
        "DISPOSITIVOS"
    ])
    
    # ==================== TAB 1: MONITOREO EN VIVO ====================
    with tab1:
        # Imagen de fondo única para toda la pestaña
        img_path = Path(__file__).parent / 'assets' / 'microalgas.webp'
        img_b64 = get_image_base64(img_path) if img_path.exists() else None
        
        if img_b64:
            st.markdown(f"""
                <style>
                #bg-microalgas {{
                    position: fixed;
                    top: 0;
                    left: 0;
                    width: 100vw;
                    height: 100vh;
                    z-index: 0;
                    object-fit: cover;
                    opacity: 0.5;
                    filter: brightness(1.1) contrast(1.1) saturate(1.3);
                    pointer-events: none;
                }}
                </style>
                <img src="data:image/webp;base64,{img_b64}" id="bg-microalgas">
            """, unsafe_allow_html=True)
        
        st.markdown("""
            <style>
            /* Asegurar que el contenido esté visible sobre la imagen */
            .main .block-container {
                position: relative;
                z-index: 1;
            }
            
            .metrics-hero {
                display: flex;
                align-items: center;
                justify-content: center;
                margin-bottom: 15px;
                position: relative;
                z-index: 2;
            }
            
            .metrics-container {
                background: rgba(0, 217, 255, 0.08);
                border: 2px solid rgba(0, 217, 255, 0.3);
                border-radius: 15px;
                padding: 20px;
                backdrop-filter: blur(20px);
                box-shadow: 0 8px 32px rgba(0, 217, 255, 0.3);
            }
            
            .section-title {
                text-align: center;
                font-size: 1.8rem;
                color: #FFFFFF;
                margin-bottom: 20px;
                text-shadow: 0 0 20px rgba(0, 217, 255, 0.5);
            }
            </style>
        """, unsafe_allow_html=True)

        # Header principal
        st.markdown("""
            <div class="main-header">
                <h1>Sistema de Monitoreo Ambiental en Tiempo Real</h1>
                <p>Control y Análisis de Parámetros Físico-Químicos</p>
            </div>
        """, unsafe_allow_html=True)

        # SECCIÓN 1: MÉTRICAS PRINCIPALES
        st.markdown('<div class="metrics-hero">', unsafe_allow_html=True)
        st.markdown('<div class="metrics-container">', unsafe_allow_html=True)
        
        # Título de métricas
        st.markdown('<h2 class="section-title">Métricas en Tiempo Real</h2>', unsafe_allow_html=True)
        
        # Métricas en 4 columnas
        col1, col2, col3, col4 = st.columns(4)
        
        with col1:
            ph_actual = df['ph'].iloc[-1]
            ph_promedio = df['ph'].mean()
            st.metric(
                label="pH Actual",
                value=f"{ph_actual:.2f}",
                delta=f"{ph_actual - ph_promedio:.2f} vs promedio"
            )
        
        with col2:
            ph_min = df['ph'].min()
            ph_max = df['ph'].max()
            st.metric(
                label="Rango pH",
                value=f"{ph_min:.2f} - {ph_max:.2f}",
                delta=f"Δ {ph_max - ph_min:.2f}"
            )
        
        with col3:
            temp_actual = df['temperatura'].iloc[-1]
            temp_promedio = df['temperatura'].mean()
            st.metric(
                label="Temp. Actual",
                value=f"{temp_actual:.1f}°C",
                delta=f"{temp_actual - temp_promedio:.1f}°C vs promedio"
            )
        
        with col4:
            temp_min = df['temperatura'].min()
            temp_max = df['temperatura'].max()
            st.metric(
                label="Rango Temp.",
                value=f"{temp_min:.1f} - {temp_max:.1f}°C",
                delta=f"Δ {temp_max - temp_min:.1f}°C"
            )
        
        st.markdown('</div>', unsafe_allow_html=True)
        st.markdown('</div>', unsafe_allow_html=True)
        
        # SECCIÓN 2: GRÁFICOS
        st.markdown("""
            <hr style="border: none; height: 2px; background: linear-gradient(90deg, transparent, rgba(255,255,255,0.6), transparent); margin: 50px 0 30px 0;">
            <div style="margin-bottom: 20px;">
                <h2 style="text-align: center; color: #00D9FF; font-size: 2.5rem; text-shadow: 0 0 20px rgba(0, 217, 255, 0.5);">
                    Visualización de Datos
                </h2>
                <p style="text-align: center; color: #FFFFFF; font-size: 1rem; margin-top: 5px;">
        """, unsafe_allow_html=True)
        
        # Filtrar datos a últimos 5 minutos para gráficos
        tiempo_limite = df['timestamp'].max() - timedelta(minutes=5)
        df_5min = df[df['timestamp'] >= tiempo_limite].copy()
        
        # Gráficos de series temporales
        col_izq, col_der = st.columns(2)
        
        with col_izq:
            st.markdown("<h3 style='text-align: center; color: #00D9FF;'>pH en Tiempo Real</h3>", unsafe_allow_html=True)
            fig_ph = go.Figure()
            
            # Calcular rango dinámico para pH
            ph_min = df_5min['ph'].min()
            ph_max = df_5min['ph'].max()
            ph_range = ph_max - ph_min
            ph_margin = max(0.5, ph_range * 0.2)  # Margen mínimo de 0.5 o 20% del rango
            
            fig_ph.add_trace(go.Scatter(
                x=df_5min['timestamp'],
                y=df_5min['ph'],
                mode='lines+markers',
                name='pH',
                line=dict(color=COLORS['ph'], width=1.5),
                marker=dict(size=4, color=COLORS['ph'])
            ))
            
            fig_ph.add_hline(
                y=7.0,
                line_dash="dash",
                line_color=COLORS['success'],
                annotation_text="pH Neutro (7.0)",
                annotation_position="right"
            )
            
            fig_ph.update_layout(
                height=350,
                plot_bgcolor='rgba(15, 32, 39, 0.7)',
                paper_bgcolor='rgba(15, 32, 39, 0.7)',
                font=dict(color='white'),
                xaxis=dict(
                    title="Tiempo",
                    gridcolor='rgba(255,255,255,0.1)',
                    showgrid=True
                ),
                yaxis=dict(
                    title="pH",
                    gridcolor='rgba(255,255,255,0.1)',
                    showgrid=True,
                    range=[ph_min - ph_margin, ph_max + ph_margin]
                ),
                hovermode='x unified'
            )
            st.plotly_chart(fig_ph, width='stretch')
        
        with col_der:
            st.markdown("<h3 style='text-align: center; color: #00D9FF;'>Temperatura en Tiempo Real</h3>", unsafe_allow_html=True)
            fig_temp = go.Figure()
            
            # Calcular rango dinámico para temperatura
            temp_min = df_5min['temperatura'].min()
            temp_max = df_5min['temperatura'].max()
            temp_range = temp_max - temp_min
            temp_margin = max(1.0, temp_range * 0.2)  # Margen mínimo de 1°C o 20% del rango
            
            fig_temp.add_trace(go.Scatter(
                x=df_5min['timestamp'],
                y=df_5min['temperatura'],
                mode='lines+markers',
                name='Temperatura',
                line=dict(color=COLORS['temperatura'], width=1.5),
                marker=dict(size=4, color=COLORS['temperatura'])
            ))
            
            fig_temp.update_layout(
                height=350,
                plot_bgcolor='rgba(15, 32, 39, 0.7)',
                paper_bgcolor='rgba(15, 32, 39, 0.7)',
                font=dict(color='white'),
                xaxis=dict(
                    title="Tiempo",
                    gridcolor='rgba(255,255,255,0.1)',
                    showgrid=True
                ),
                yaxis=dict(
                    title="Temperatura (°C)",
                    gridcolor='rgba(255,255,255,0.1)',
                    showgrid=True,
                    range=[temp_min - temp_margin, temp_max + temp_margin]
                ),
                hovermode='x unified'
            )
            st.plotly_chart(fig_temp, width='stretch')

    # ==================== TAB 2: ANÁLISIS ESTADÍSTICO ====================
    with tab2:
        # Imagen de fondo única para esta pestaña
        img_tab2_path = Path(__file__).parent / 'assets' / 'alga marina.jpg'
        img_tab2_b64 = get_image_base64(img_tab2_path) if img_tab2_path.exists() else None
        
        if img_tab2_b64:
            st.markdown(f"""
                <style>
                #bg-alga-marina {{
                    position: fixed;
                    top: 0;
                    left: 0;
                    width: 100vw;
                    height: 100vh;
                    z-index: 0;
                    object-fit: cover;
                    opacity: 0.5;
                    filter: brightness(1.1) contrast(1.1) saturate(1.3);
                    pointer-events: none;
                }}
                </style>
                <img src="data:image/jpeg;base64,{img_tab2_b64}" id="bg-alga-marina">
            """, unsafe_allow_html=True)
        
        st.markdown("### 📊 Análisis Estadístico Detallado")
        
        # Análisis estadístico
        col_stat1, col_stat2 = st.columns(2)
        
        with col_stat1:
            st.markdown("#### 🧪 Distribución de pH")
            fig_hist_ph = go.Figure()
            
            fig_hist_ph.add_trace(go.Histogram(
                x=df['ph'],
                nbinsx=30,
                marker=dict(
                    color=COLORS['ph'],
                    line=dict(color='white', width=1)
                ),
                name='pH'
            ))
            
            fig_hist_ph.update_layout(
                height=350,
                plot_bgcolor='rgba(0,0,0,0)',
                paper_bgcolor='rgba(0,0,0,0)',
                font=dict(color='white'),
                xaxis=dict(title="pH", gridcolor='rgba(255,255,255,0.1)'),
                yaxis=dict(title="Frecuencia", gridcolor='rgba(255,255,255,0.1)'),
                showlegend=False
            )
            st.plotly_chart(fig_hist_ph, width='stretch')
            
            # Estadísticas pH
            st.markdown("##### Estadísticas pH")
            col_ph1, col_ph2, col_ph3 = st.columns(3)
            with col_ph1:
                st.metric("Media", f"{df['ph'].mean():.2f}")
            with col_ph2:
                st.metric("Mediana", f"{df['ph'].median():.2f}")
            with col_ph3:
                st.metric("Desv. Est.", f"{df['ph'].std():.2f}")
        
        with col_stat2:
            st.markdown("#### 🌡 Distribución de Temperatura")
            fig_hist_temp = go.Figure()
            
            fig_hist_temp.add_trace(go.Histogram(
                x=df['temperatura'],
                nbinsx=30,
                marker=dict(
                    color=COLORS['temperatura'],
                    line=dict(color='white', width=1)
                ),
                name='Temperatura'
            ))
            
            fig_hist_temp.update_layout(
                height=350,
                plot_bgcolor='rgba(0,0,0,0)',
                paper_bgcolor='rgba(0,0,0,0)',
                font=dict(color='white'),
                xaxis=dict(title="Temperatura (°C)", gridcolor='rgba(255,255,255,0.1)'),
                yaxis=dict(title="Frecuencia", gridcolor='rgba(255,255,255,0.1)'),
                showlegend=False
            )
            st.plotly_chart(fig_hist_temp, width='stretch')
            
            # Estadísticas Temperatura
            st.markdown("##### Estadísticas Temperatura")
            col_temp1, col_temp2, col_temp3 = st.columns(3)
            with col_temp1:
                st.metric("Media", f"{df['temperatura'].mean():.1f}°C")
            with col_temp2:
                st.metric("Mediana", f"{df['temperatura'].median():.1f}°C")
            with col_temp3:
                st.metric("Desv. Est.", f"{df['temperatura'].std():.1f}°C")
        
        st.markdown("---")
        
        # Gráfico de caja (Box plot)
        st.markdown("### 📦 Gráficos de Caja")
        col_box1, col_box2 = st.columns(2)
        
        with col_box1:
            fig_box_ph = go.Figure()
            fig_box_ph.add_trace(go.Box(
                y=df['ph'],
                name='pH',
                marker_color=COLORS['ph'],
                boxmean='sd'
            ))
            fig_box_ph.update_layout(
                height=300,
                plot_bgcolor='rgba(0,0,0,0)',
                paper_bgcolor='rgba(0,0,0,0)',
                font=dict(color='white'),
                yaxis=dict(title="pH", gridcolor='rgba(255,255,255,0.1)'),
                showlegend=False
            )
            st.plotly_chart(fig_box_ph, width='stretch')
        
        with col_box2:
            fig_box_temp = go.Figure()
            fig_box_temp.add_trace(go.Box(
                y=df['temperatura'],
                name='Temperatura',
                marker_color=COLORS['temperatura'],
                boxmean='sd'
            ))
            fig_box_temp.update_layout(
                height=300,
                plot_bgcolor='rgba(0,0,0,0)',
                paper_bgcolor='rgba(0,0,0,0)',
                font=dict(color='white'),
                yaxis=dict(title="Temperatura (°C)", gridcolor='rgba(255,255,255,0.1)'),
                showlegend=False
            )
            st.plotly_chart(fig_box_temp, width='stretch')
    
    # ==================== TAB 3: REGISTROS ====================
    with tab3:
        st.markdown("### 📋 Tabla de Registros Completa")
        
        # Filtros
        col_f1, col_f2 = st.columns(2)
        with col_f1:
            mostrar_registros = st.selectbox(
                "Mostrar",
                options=[10, 25, 50, 100, "Todos"],
                index=1
            )
        
        # Preparar datos para tabla
        df_display = df[['timestamp', 'ph', 'temperatura']].copy()
        df_display['timestamp'] = df_display['timestamp'].dt.strftime('%Y-%m-%d %H:%M:%S')
        df_display = df_display.sort_values('timestamp', ascending=False)
        df_display.columns = ['Fecha y Hora', 'pH', 'Temperatura (°C)']
        
        # Mostrar según selección
        if mostrar_registros == "Todos":
            st.dataframe(df_display, width='stretch', hide_index=True, height=600)
        else:
            st.dataframe(df_display.head(mostrar_registros), width='stretch', hide_index=True, height=600)
        
        # Botón de descarga
        csv = df_display.to_csv(index=False).encode('utf-8')
        st.download_button(
            label="⬇ Descargar CSV",
            data=csv,
            file_name=f'datos_sensores_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv',
            mime='text/csv',
        )
    
    # ==================== TAB 4: DISPOSITIVOS ====================
    with tab4:
        st.markdown("### 🔧 Información de Dispositivos")
        
        col_dev1, col_dev2 = st.columns(2)
        
        with col_dev1:
            st.markdown("#### 🌊 Sensor de pH")
            st.info("""
            **Modelo:** Sensor pH Analógico  
            **Rango:** 0 - 14 pH  
            **Precisión:** ±0.1 pH  
            **Estado:**  Operativo
            """)
            
            st.metric("Última Lectura", f"{df['ph'].iloc[-1]:.2f} pH")
            st.metric("Total de Lecturas", len(df))
            
        with col_dev2:
            st.markdown("#### 🌡 Sensor de Temperatura")
            st.info("""
            **Modelo:** DHT22 / DS18B20  
            **Rango:** -40°C a 80°C  
            **Precisión:** ±0.5°C  
            **Estado:**  Operativo
            """)
            
            st.metric("Última Lectura", f"{df['temperatura'].iloc[-1]:.1f}°C")
            st.metric("Total de Lecturas", len(df))
        
        st.markdown("---")
        
        st.markdown("#### 📡 Conexión y Base de Datos")
        col_info1, col_info2, col_info3 = st.columns(3)
        
        with col_info1:
            st.metric("Base de Datos", os.getenv('MONGO_DB'))
        with col_info2:
            st.metric("Colección", os.getenv('MONGO_COLLECTION'))
        with col_info3:
            st.metric("Estado Conexión", " Conectado")
    
else:
    st.warning(" No hay datos disponibles para el rango temporal seleccionado")
    st.info(f"Intentando conectar a: {os.getenv('MONGO_DB')}/{os.getenv('MONGO_COLLECTION')}")

# Auto-refresh automático
if auto_refresh:
    import time
    time.sleep(60)
    st.rerun()
