"""Componente del sidebar con controles"""
import streamlit as st
from config.settings import Settings
from database import MongoHandler

def render_sidebar(df):
    """
    Renderiza el sidebar con controles y métricas
    
    Args:
        df: DataFrame con los datos cargados
        
    Returns:
        Tuple (rango_horas, auto_refresh)
    """
    st.sidebar.markdown("### Panel de Control")
    
    # Selector de rango temporal
    rango_horas = st.sidebar.selectbox(
        "Rango Temporal",
        options=Settings.RANGO_HORAS_OPTIONS,
        index=0,
        format_func=lambda x: f"Última{'s' if x > 1 else ''} {x} hora{'s' if x > 1 else ''}"
    )
    
    # Botón de actualización
    if st.sidebar.button(" Actualizar Datos", type="primary", use_container_width=True):
        MongoHandler.limpiar_cache()
        st.rerun()
    
    
    # Auto-refresh siempre activo
    st.sidebar.info("🔄 Actualizando automáticamente cada 10 segundos...")
    auto_refresh = True
    
    st.sidebar.markdown("---")
    
    # Información de conexión (si hay datos)
    if not df.empty:
        st.sidebar.success(f"Conectado a MongoDB")
        st.sidebar.metric("Registros Cargados", len(df))
        st.sidebar.metric("Último Registro", 
                         df['timestamp'].max().strftime("%H:%M:%S"))
    else:
        st.sidebar.warning("Sin datos disponibles")
    
    return rango_horas, auto_refresh
