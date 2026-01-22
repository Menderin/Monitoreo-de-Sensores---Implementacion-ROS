"""Página de Monitoreo en Vivo"""
import streamlit as st
import textwrap
from pathlib import Path
from datetime import timedelta
from utils import get_image_base64
from components import crear_grafico_lineas
from config import Settings
from database import MongoHandler
from styles import apply_metric_styles, apply_chart_styles

def render_monitoreo_vivo(df_inicial, rango_horas):
    """Renderiza la página de monitoreo en tiempo real"""
    
    # Imagen de fondo (solo una vez, no se recarga)
    img_path = Path(__file__).parent.parent / 'assets' / 'microalgas.webp'
    img_b64 = get_image_base64(img_path) if img_path.exists() else None
    
    # CSS: Fondo y Ajustes de Espaciado (OPTIMIZADO)
    if img_b64:
        st.markdown(textwrap.dedent(f"""
            <style>
            /* --- FORZAR: SIN SCROLL --- */
            html, body {{
                overflow: hidden !important;
                height: 100vh !important;
            }}
            
            /* Fondo fijo de microalgas */
            #bg-microalgas {{
                position: fixed;
                top: 0;
                left: 0;
                width: 100vw;
                height: 100vh;
                z-index: -1;
                object-fit: cover;
                opacity: 0.5;
                filter: brightness(1.1) contrast(1.1) saturate(1.3);
                pointer-events: none;
            }}
            
            #bg-microalgas ~ #bg-microalgas {{ display: none !important; }}
            
            [data-testid="stAppViewContainer"] {{
                position: relative;
                z-index: 1;
            }}
            
            /* --- ELIMINAR TODO PADDING/MARGIN --- */
            [data-testid="stAppViewContainer"] > .main > div:first-child {{
                padding-top: 0px !important;
                padding-bottom: 0px !important;
                margin-top: 0px !important;
                margin-bottom: 0px !important;
            }}
            
            /* Sin espacio antes de los gráficos */
            .series-temporales-section {{
                padding-top: 0px !important;
                margin-top: 0px !important;
            }}
            
            /* Eliminar padding del contenedor principal */
            .main .block-container {{
                padding-top: 0 !important;
                padding-bottom: 0 !important;
            }}
            </style>
            <img id="bg-microalgas" src="data:image/webp;base64,{img_b64}">
        """), unsafe_allow_html=True)
    
    # Header (estático)
    st.markdown("""
        <div class="main-header" style="margin-bottom: 5px; margin-top: 5px;">
            <h1>🌊 Monitoreo en Vivo</h1>
            <p>Visualización en tiempo real de sensores ambientales</p>
        </div>
    """, unsafe_allow_html=True)
    
    # Fragment que se actualiza automáticamente cada 10 segundos
    @st.fragment(run_every=Settings.AUTO_REFRESH_INTERVAL)
    def metricas_y_graficos():
        """Sección que se actualiza automáticamente"""
        # Recargar datos frescos
        df = MongoHandler.cargar_datos(horas=rango_horas)
        
        if df.empty:
            st.warning("No hay datos disponibles")
            return
        
        # Filtrar datos a últimos 5 minutos para gráficos
        tiempo_limite = df['timestamp'].max() - timedelta(minutes=5)
        df_5min = df[df['timestamp'] >= tiempo_limite].copy()
        
        # Aplicar estilos personalizados
        apply_metric_styles()
        apply_chart_styles()
        
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
            ph_mean_current = df['ph'].mean()
            ph_mean_previous = df['ph'].iloc[:len(df)//2].mean() if len(df) > 2 else ph_mean_current
            ph_delta = ph_mean_current - ph_mean_previous
            
            st.metric(
                label="pH Promedio",
                value=f"{ph_mean_current:.2f}",
                delta=f"{ph_delta:.2f}" if len(df) > 2 else None
            )
        
        with col4:
            temp_mean_current = df['temperatura'].mean()
            temp_mean_previous = df['temperatura'].iloc[:len(df)//2].mean() if len(df) > 2 else temp_mean_current
            temp_delta = temp_mean_current - temp_mean_previous
            
            st.metric(
                label="Temp. Promedio",
                value=f"{temp_mean_current:.1f}°C",
                delta=f"{temp_delta:.1f}°C" if len(df) > 2 else None
            )
        
        # Separador ultra compacto
        st.markdown("<hr style='border: 1px solid rgba(255,255,255,0.3); margin: 5px 0;'>", unsafe_allow_html=True)
        
        # Gráficos de series temporales
        st.markdown("""
            <div class="series-temporales-section">
                <h2 style='color: white; font-size: 1.8rem; margin-bottom: 1rem;'>
                    Series Temporales en vivo - Últimos 5 minutos
                </h2>
            </div>
        """, unsafe_allow_html=True)
        
        col_ph, col_temp = st.columns(2)
        
        with col_ph:
            fig_ph = crear_grafico_lineas(
                df_5min, 'ph', 
                'Niveles de pH en el Tiempo',
                'ph',
                'pH'
            )
            st.plotly_chart(fig_ph, use_container_width=True)
        
        with col_temp:
            fig_temp = crear_grafico_lineas(
                df_5min, 'temperatura',
                'Temperatura en el Tiempo',
                'temperatura',
                'Temperatura (°C)'
            )
            st.plotly_chart(fig_temp, use_container_width=True)
        
        st.markdown('</div>', unsafe_allow_html=True)
    
    # Ejecutar el fragment
    metricas_y_graficos()