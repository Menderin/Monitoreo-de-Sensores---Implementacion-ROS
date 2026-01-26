"""Página de Monitoreo en Vivo (Selector Izquierdo + Solo Online + Glassmorphism)"""
import streamlit as st
import textwrap
from pathlib import Path
from datetime import datetime, timedelta, timezone
import dateutil.parser
from utils import get_image_base64
from components import crear_grafico_lineas
from config import Settings
from database.mongo_handler import MongoHandler, obtener_dispositivos
from styles import apply_metric_styles, apply_chart_styles

def is_device_online(ultima_conexion_str, umbral_minutos=5):
    """Verifica si el dispositivo ha enviado datos recientemente"""
    if not ultima_conexion_str:
        return False
    try:
        ultima = dateutil.parser.isoparse(str(ultima_conexion_str))
        if ultima.tzinfo is None:
            ultima = ultima.replace(tzinfo=timezone.utc)
        
        ahora = datetime.now(timezone.utc)
        diferencia = (ahora - ultima).total_seconds() / 60
        return diferencia < umbral_minutos
    except:
        return False

def render_monitoreo_vivo(df_inicial, rango_horas):
    """Renderiza la página de monitoreo en tiempo real"""
    
    # Imagen de fondo
    img_path = Path(__file__).parent.parent / 'assets' / 'microalgas.webp'
    img_b64 = get_image_base64(img_path) if img_path.exists() else None
    
    # CSS: Fondo y Ajustes de Estilo
    if img_b64:
        st.markdown(textwrap.dedent(f"""
            <style>
            html, body {{ overflow: hidden !important; height: 100vh !important; }}
            
            #bg-microalgas {{
                position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
                z-index: -1; object-fit: cover; opacity: 0.5;
                filter: brightness(1.1) contrast(1.1) saturate(1.3); pointer-events: none;
            }}
            #bg-microalgas ~ #bg-microalgas {{ display: none !important; }}
            
            [data-testid="stAppViewContainer"] {{ position: relative; z-index: 1; }}
            
            /* Ajustes de espaciado */
            [data-testid="stAppViewContainer"] > .main > div:first-child {{
                padding-top: 0px !important; margin-top: 0px !important;
            }}
            .main .block-container {{ padding-top: 1rem !important; padding-bottom: 0 !important; }}
            
            /* --- ESTILO SELECTOR SEMITRANSPARENTE (GLASSMORPHISM OSCURO) --- */
            /* Contenedor del selector */
            div[data-testid="stSelectbox"] > div > div {{
                background-color: rgba(0, 0, 0, 0.4) !important; /* Negro semitransparente */
                backdrop-filter: blur(5px) !important; /* Efecto vidrio */
                border: 1px solid rgba(255, 255, 255, 0.2) !important;
                color: white !important;
                border-radius: 5px !important;
            }}
            
            /* Texto y flecha dentro del selector */
            div[data-testid="stSelectbox"] div[data-baseweb="select"] span {{
                color: white !important;
            }}
            div[data-testid="stSelectbox"] svg {{
                fill: white !important;
            }}
            
            /* Menú desplegable */
            div[data-baseweb="popover"] {{
                background-color: rgba(20, 20, 20, 0.95) !important;
                border: 1px solid rgba(255, 255, 255, 0.2) !important;
            }}
            </style>
            <img id="bg-microalgas" src="data:image/webp;base64,{img_b64}">
        """), unsafe_allow_html=True)
    
    # ---------------------------------------------------------
    # 1. ENCABEZADO (Texto Centrado)
    # ---------------------------------------------------------
    st.markdown("""
        <div class="main-header" style="margin-bottom: 15px; margin-top: 5px; text-align: center;">
            <h1>🌊 Monitoreo en Vivo</h1>
            <p style="margin-bottom: 10px;">Visualización en tiempo real de sensores ambientales</p>
        </div>
    """, unsafe_allow_html=True)
    
    # ---------------------------------------------------------
    # 2. LÓGICA DE DISPOSITIVOS ONLINE
    # ---------------------------------------------------------
    lista_metadata = obtener_dispositivos()
    dispositivos_online = []
    
    # Filtramos solo los que están activos (< 5 min)
    for d in lista_metadata:
        last_seen = d.get('conexion', {}).get('ultima')
        if is_device_online(last_seen, umbral_minutos=5):
            dispositivos_online.append(d)
            
    # Ordenar por actividad más reciente
    dispositivos_online.sort(key=lambda x: x.get('conexion', {}).get('ultima', ''), reverse=True)
    
    mapa_nombres = {d['_id']: d.get('nombre', d['_id']) for d in dispositivos_online}
    opciones = list(mapa_nombres.keys())

    # ---------------------------------------------------------
    # 3. SELECTOR (Debajo del título, Alineado a la Izquierda)
    # ---------------------------------------------------------
    # Usamos columnas: Columna 1 (estrecha) para el selector, Columna 2 (ancha) vacía.
    c_selector, c_espacio = st.columns([1, 2]) 
    
    with c_selector:
        if opciones:
            id_seleccionado = st.selectbox(
                "Seleccionar Dispositivo", # Label oculto por visibility
                options=opciones,
                format_func=lambda x: f"🟢 {mapa_nombres[x]}",
                label_visibility="collapsed"
            )
        else:
            # Si no hay dispositivos online, mostramos un aviso deshabilitado
            st.warning("⚠️ Sin dispositivos online")
            id_seleccionado = None

    # ---------------------------------------------------------
    # 4. FRAGMENTO DE ACTUALIZACIÓN
    # ---------------------------------------------------------
    @st.fragment(run_every=Settings.AUTO_REFRESH_INTERVAL)
    def metricas_y_graficos(device_id):
        
        # Si no hay selección (porque no hay online), detenemos la renderización
        if device_id is None:
            return

        df = MongoHandler.cargar_datos(horas=rango_horas)
        
        if df.empty:
            st.info("Esperando datos...")
            return
        
        # Filtrar por dispositivo seleccionado
        df_device = df[df['dispositivo_id'] == device_id].copy()
        
        if df_device.empty:
            st.warning("Dispositivo en línea, pero sin datos históricos recientes.")
            return

        # Filtrar últimos 5 minutos para gráficos
        tiempo_limite = df_device['timestamp'].max() - timedelta(minutes=5)
        df_5min = df_device[df_device['timestamp'] >= tiempo_limite].copy()
        
        apply_metric_styles()
        apply_chart_styles()
        
        # --- MÉTRICAS (Manteniendo textos y tamaños originales) ---
        col1, col2, col3, col4 = st.columns(4)
        ultimo = df_device.iloc[-1]
        penultimo = df_device.iloc[-2] if len(df_device) > 1 else ultimo
        
        with col1:
            st.metric(
                label="pH Actual",
                value=f"{ultimo['ph']:.2f}",
                delta=f"{ultimo['ph'] - penultimo['ph']:.2f}"
            )
        with col2:
            st.metric(
                label="Temperatura Actual",
                value=f"{ultimo['temperatura']:.1f}°C",
                delta=f"{ultimo['temperatura'] - penultimo['temperatura']:.1f}°C"
            )
        with col3:
            ph_mean = df_device['ph'].mean()
            ph_mean_prev = df_device['ph'].iloc[:len(df_device)//2].mean() if len(df_device) > 2 else ph_mean
            st.metric(
                label="pH Promedio",
                value=f"{ph_mean:.2f}",
                delta=f"{ph_mean - ph_mean_prev:.2f}" if len(df_device) > 10 else None
            )
        with col4:
            temp_mean = df_device['temperatura'].mean()
            temp_mean_prev = df_device['temperatura'].iloc[:len(df_device)//2].mean() if len(df_device) > 2 else temp_mean
            st.metric(
                label="Temp. Promedio",
                value=f"{temp_mean:.1f}°C",
                delta=f"{temp_mean - temp_mean_prev:.1f}°C" if len(df_device) > 10 else None
            )
        
        st.markdown("<hr style='border: 1px solid rgba(255,255,255,0.3); margin: 5px 0;'>", unsafe_allow_html=True)
        
        # --- GRÁFICOS ---
        st.markdown("""
            <div class="series-temporales-section">
                <h2 style='color: white; font-size: 1.8rem; margin-bottom: 1rem;'>
                    Series Temporales en vivo - Últimos 5 minutos
                </h2>
            </div>
        """, unsafe_allow_html=True)
        
        col_ph, col_temp = st.columns(2)
        
        with col_ph:
            fig_ph = crear_grafico_lineas(df_5min, 'ph', 'Niveles de pH en el Tiempo', 'ph', 'pH')
            st.plotly_chart(fig_ph, use_container_width=True)
        
        with col_temp:
            fig_temp = crear_grafico_lineas(df_5min, 'temperatura', 'Temperatura en el Tiempo', 'temperatura', 'Temperatura (°C)')
            st.plotly_chart(fig_temp, use_container_width=True)
        
        st.markdown('</div>', unsafe_allow_html=True)
    
    metricas_y_graficos(id_seleccionado)