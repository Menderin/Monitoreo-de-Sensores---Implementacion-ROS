"""Página de Análisis Estadístico"""
import streamlit as st
from pathlib import Path
from utils import get_image_base64
from components import crear_grafico_caja

def render_analisis_estadistico(df):
    """Renderiza la página de análisis estadístico"""

    # Imagen de fondo
    img_path = Path(__file__).parent.parent / 'assets' / 'alga marina.webp'
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
            
            /* Asegurar que el contenido esté visible sobre la imagen */
            .main .block-container {{
                position: relative;
                z-index: 1;
            }}
            </style>
            <img id="bg-alga marina" src="data:image/webp;base64,{img_b64}">
        """, unsafe_allow_html=True)
    
    st.markdown("### 📊 Análisis Estadístico Completo")
    
    # Estadísticas descriptivas
    st.markdown("#### 📈 Estadísticas Descriptivas")
    
    col_stats1, col_stats2 = st.columns(2)
    
    with col_stats1:
        st.markdown("##### 🧪 pH")
        col_ph1, col_ph2, col_ph3 = st.columns(3)
        with col_ph1:
            st.metric("Media", f"{df['ph'].mean():.2f}")
        with col_ph2:
            st.metric("Mediana", f"{df['ph'].median():.2f}")
        with col_ph3:
            st.metric("Desv. Est.", f"{df['ph'].std():.2f}")
        
        col_ph4, col_ph5 = st.columns(2)
        with col_ph4:
            st.metric("Mínimo", f"{df['ph'].min():.2f}")
        with col_ph5:
            st.metric("Máximo", f"{df['ph'].max():.2f}")
    
    with col_stats2:
        st.markdown("##### 🌡 Temperatura")
        col_temp1, col_temp2, col_temp3 = st.columns(3)
        with col_temp1:
            st.metric("Media", f"{df['temperatura'].mean():.1f}°C")
        with col_temp2:
            st.metric("Mediana", f"{df['temperatura'].median():.1f}°C")
        with col_temp3:
            st.metric("Desv. Est.", f"{df['temperatura'].std():.1f}°C")
        
        col_temp4, col_temp5 = st.columns(2)
        with col_temp4:
            st.metric("Mínimo", f"{df['temperatura'].min():.1f}°C")
        with col_temp5:
            st.metric("Máximo", f"{df['temperatura'].max():.1f}°C")
    
    st.markdown("---")
    
    # Gráficos de caja (Box plots)
    st.markdown("### 📦 Gráficos de Caja")
    col_box1, col_box2 = st.columns(2)
    
    with col_box1:
        fig_box_ph = crear_grafico_caja(
            df, 'ph',
            'pH',
            'ph',
            'pH'
        )
        st.plotly_chart(fig_box_ph, use_container_width=True)
    
    with col_box2:
        fig_box_temp = crear_grafico_caja(
            df, 'temperatura',
            'Temperatura',
            'temperatura',
            'Temperatura (°C)'
        )
        st.plotly_chart(fig_box_temp, use_container_width=True)
