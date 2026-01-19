"""Página de Monitoreo en Vivo"""
import streamlit as st
from pathlib import Path
from datetime import timedelta
from utils import get_image_base64
from components import crear_grafico_lineas

def render_monitoreo_vivo(df):
    """Renderiza la página de monitoreo en tiempo real"""
    
    # Filtrar datos a últimos 5 minutos para gráficos
    tiempo_limite = df['timestamp'].max() - timedelta(minutes=5)
    df_5min = df[df['timestamp'] >= tiempo_limite].copy()
    
    # Imagen de fondo
    img_path = Path(__file__).parent.parent / 'assets' / 'microalgas.webp'
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
            <img id="bg-microalgas" src="data:image/webp;base64,{img_b64}">
        """, unsafe_allow_html=True)
    
    # CSS para hacer que la sección superior ocupe toda la pantalla
    st.markdown("""
        <style>
        /* Agregar espacio superior para empujar el contenido hacia abajo */
        [data-testid="stAppViewContainer"] > .main > div:first-child {
            padding-top: 15vh !important;
            padding-bottom: 35vh !important;
        }
        
        /* Asegurar que la sección de gráficos comience en una nueva "página" */
        .series-temporales-section {
            padding-top: 10vh;
        }
        </style>
    """, unsafe_allow_html=True)
    
    # Header
    st.markdown("""
        <div class="main-header">
            <h1>🌊 Monitoreo en Vivo</h1>
            <p>Visualización en tiempo real de sensores ambientales</p>
        </div>
    """, unsafe_allow_html=True)
    
    # Métricas principales
    col1, col2, col3, col4 = st.columns(4)
    
    with col1:
        st.metric(
            label="pH Actual",
            value=f"{df['ph'].iloc[-1]:.2f}",
            delta=f"{df['ph'].iloc[-1] - df['ph'].iloc[-2]:.2f}" if len(df) > 1 else None
        )
    
    with col2:
        st.metric(
            label="Temperatura Actual",
            value=f"{df['temperatura'].iloc[-1]:.1f}°C",
            delta=f"{df['temperatura'].iloc[-1] - df['temperatura'].iloc[-2]:.1f}°C" if len(df) > 1 else None
        )
    
    with col3:
        st.metric(
            label="pH Promedio",
            value=f"{df['ph'].mean():.2f}"
        )
    
    with col4:
        st.metric(
            label="Temp. Promedio",
            value=f"{df['temperatura'].mean():.1f}°C"
        )
    
    st.markdown("---")
    
    # Gráficos de series temporales (últimos 5 minutos)
    st.markdown('<div class="series-temporales-section">', unsafe_allow_html=True)
    st.markdown("### Series Temporales (Últimos 5 Minutos)")
    
    col_ph, col_temp = st.columns(2)
    
    with col_ph:
        fig_ph = crear_grafico_lineas(
            df_5min, 'ph', 
            'Niveles de pH en el Tiempo',
            'ph',
            'pH'
        )
        st.plotly_chart(fig_ph, width='stretch')
    
    with col_temp:
        fig_temp = crear_grafico_lineas(
            df_5min, 'temperatura',
            'Temperatura en el Tiempo',
            'temperatura',
            'Temperatura (°C)'
        )
        st.plotly_chart(fig_temp, width='stretch')
    
    st.markdown('</div>', unsafe_allow_html=True)
