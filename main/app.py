"""
Dashboard de Monitoreo Ambiental - Aplicación Principal
Sistema modular de visualización de datos de sensores
"""

import streamlit as st
import time

# Importar módulos propios
from config import Settings
from styles import apply_custom_styles
from database import MongoHandler
from components import render_sidebar
from pages import (
    render_monitoreo_vivo,
    render_analisis_estadistico,
    render_registros,
    render_dispositivos
)

# Configuración de la página
st.set_page_config(
    page_title=Settings.PAGE_TITLE,
    page_icon=Settings.PAGE_ICON,
    layout=Settings.LAYOUT,
    initial_sidebar_state=Settings.SIDEBAR_STATE
)

# Aplicar estilos CSS
apply_custom_styles()

def main():
    """Función principal de la aplicación"""
    
    # Renderizar sidebar y obtener configuración
    # Cargar datos inicialmente con valor por defecto
    df = MongoHandler.cargar_datos(horas=1)
    rango_horas, auto_refresh = render_sidebar(df)
    
    # Recargar datos con el rango seleccionado
    df = MongoHandler.cargar_datos(horas=rango_horas)
    
    if not df.empty:
        # Crear pestañas de navegación
        tab1, tab2, tab3, tab4 = st.tabs([
            "📊 MONITOREO EN VIVO",
            "📈 ANÁLISIS ESTADÍSTICO", 
            "📋 REGISTROS",
            "🔧 DISPOSITIVOS"
        ])
        
        # Renderizar cada pestaña
        with tab1:
            render_monitoreo_vivo(df)
        
        with tab2:
            render_analisis_estadistico(df)
        
        with tab3:
            render_registros(df)
        
        with tab4:
            render_dispositivos(df)
        
        # Auto-refresh si está activado
        if auto_refresh:
            time.sleep(Settings.AUTO_REFRESH_INTERVAL)
            st.rerun()
    
    else:
        st.warning("⚠️ No hay datos disponibles para el rango temporal seleccionado")
        st.info(f"Intentando conectar a: {Settings.MONGO_DB}/{Settings.MONGO_COLLECTION}")

if __name__ == "__main__":
    main()
