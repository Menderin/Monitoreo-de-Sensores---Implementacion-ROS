"""Página de Registros (Optimizada: Carga Inteligente)"""
import streamlit as st
import textwrap
import json
import base64
from pathlib import Path
from datetime import datetime
from config.settings import Settings
from utils import get_image_base64
# Importamos la nueva función
from database.mongo_handler import cargar_datos_inteligente

def load_css_file(css_file_path):
    """Lee un archivo CSS y lo inyecta en Streamlit"""
    with open(css_file_path) as f:
        st.markdown(f'<style>{f.read()}</style>', unsafe_allow_html=True)

def render_registros(df_dummy):
    """
    Renderiza la página de tabla de registros con optimización de base de datos.
    """
    
    # 1. Cargar CSS
    css_path = Path(__file__).parent.parent / 'styles' / 'custom_css_registros.py'
    if css_path.exists(): load_css_file(css_path)
        
    # 2. Fondo
    img_path = Path(__file__).parent.parent / 'assets' / 'fondo hexagonal negro.jpg'
    img_b64 = get_image_base64(img_path) if img_path.exists() else None
    if img_b64:
        st.markdown(f'<img id="bg-registros" src="data:image/webp;base64,{img_b64}">', unsafe_allow_html=True)
    
    st.markdown("### Registros e historial")
    
    # ==========================================
    # LÓGICA DE FILTRADO INTELIGENTE
    # ==========================================
    
    # Configuración: Texto -> (Límite, Horas)
    config_filtros = {
        "Últimos 50":       {"limit": 50, "hours": None},
        "Últimos 100":      {"limit": 100, "hours": None},
        "Últimos 500":      {"limit": 500, "hours": None},
        "Última Hora":      {"limit": None, "hours": 1},
        "Últimas 6 Horas":  {"limit": None, "hours": 6},
        "Últimas 24 Horas": {"limit": None, "hours": 24},
        "Todo el Historial": {"limit": None, "hours": None} 
    }
    
    col_filtro1, col_filtro2 = st.columns([2, 1])
    with col_filtro1:
        seleccion = st.radio(
            "Seleccionar Rango:", 
            options=list(config_filtros.keys()), 
            index=0, 
            horizontal=True, 
            label_visibility="visible"
        )
    
    params = config_filtros[seleccion]

    if seleccion == "Todo el Historial":
        with st.spinner("⏳ Descargando historial completo de la nube..."):
            df_display = cargar_datos_inteligente(limite=params["limit"], horas_atras=params["hours"])
    else:
        df_display = cargar_datos_inteligente(limite=params["limit"], horas_atras=params["hours"])
    
    if df_display.empty:
        st.warning("No se encontraron registros para el rango seleccionado.")
        return

    with col_filtro2:
        st.write("") 
        st.write("")
        st.info(f"Visualizando **{len(df_display)}** registros")

    df_show = df_display.copy()
    df_show['timestamp'] = df_show['timestamp'].dt.strftime('%Y-%m-%d %H:%M:%S')
    df_show.columns = ['Fecha y Hora', 'Dispositivo', 'Temperatura (°C)', 'pH']
    
    st.dataframe(df_show, use_container_width=True, hide_index=True, height=500)
    
    st.markdown("---")
    
    csv_str = df_show.to_csv(index=False).encode('utf-8')
    json_str = df_show.to_json(orient='records', indent=2).encode('utf-8')
    
    b64_csv = base64.b64encode(csv_str).decode()
    b64_json = base64.b64encode(json_str).decode()
    
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_name_csv = f'registros_{timestamp_str}.csv'
    file_name_json = f'registros_{timestamp_str}.json'
    
    st.markdown(f"""
        <div style="text-align: left; font-size: 1rem; color: #cccccc; padding: 10px 0;">
            Puede descargar los registros en formato: 
            <a href="data:text/csv;base64,{b64_csv}" download="{file_name_csv}" class="download-link csv-tag">CSV</a>
            o 
            <a href="data:application/json;base64,{b64_json}" download="{file_name_json}" class="download-link json-tag">JSON</a>
        </div>
    """, unsafe_allow_html=True)