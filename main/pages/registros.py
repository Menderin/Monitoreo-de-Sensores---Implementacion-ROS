"""Página de Registros"""
import streamlit as st
from datetime import datetime
from config.settings import Settings

def render_registros(df):
    """Renderiza la página de tabla de registros"""
    
    st.markdown("### 📋 Tabla de Registros Completa")
    
    # Filtros
    col_f1, col_f2 = st.columns(2)
    with col_f1:
        mostrar_registros = st.selectbox(
            "Mostrar",
            options=Settings.REGISTROS_OPTIONS,
            index=1
        )
    
    # Preparar datos para tabla
    df_display = df[['timestamp', 'ph', 'temperatura']].copy()
    df_display['timestamp'] = df_display['timestamp'].dt.strftime('%Y-%m-%d %H:%M:%S')
    df_display = df_display.sort_values('timestamp', ascending=False)
    df_display.columns = ['Fecha y Hora', 'pH', 'Temperatura (°C)']
    
    # Mostrar según selección
    if mostrar_registros == "Todos":
        st.dataframe(df_display, use_container_width=True, hide_index=True, height=600)
    else:
        st.dataframe(df_display.head(mostrar_registros), use_container_width=True, hide_index=True, height=600)
    
    # Botón de descarga
    csv = df_display.to_csv(index=False).encode('utf-8')
    st.download_button(
        label="⬇️ Descargar CSV",
        data=csv,
        file_name=f'datos_sensores_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv',
        mime='text/csv',
    )
