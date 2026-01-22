"""Página de Registros"""
import streamlit as st
import textwrap
import json
import base64
from pathlib import Path
from datetime import datetime, timedelta
from config.settings import Settings
from utils import get_image_base64
from database.mongo_handler import MongoHandler, cargar_todos_datos

def render_registros(df_inicial):
    """
    Renderiza la página de tabla de registros.
    """
    
    # Cargar TODOS los datos directamente desde MongoDB
    df = cargar_todos_datos()
    
    # Cargar imagen de fondo
    img_path = Path(__file__).parent.parent / 'assets' / 'fondo hexagonal negro.jpg'
    img_b64 = get_image_base64(img_path) if img_path.exists() else None
    
    # CSS: Fondo y estilos mejorados
    if img_b64:
        st.markdown(textwrap.dedent(f"""
            <style>
            /* Fondo fijo */
            #bg-registros {{
                position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
                z-index: -1; object-fit: cover; opacity: 0.6; pointer-events: none;
            }}
            #bg-registros ~ #bg-registros {{ display: none !important; }}
            [data-testid="stAppView1Container"] {{ position: relative; z-index: 1; }}
            
            /* --- ESTILOS GENERALES --- */
            
            /* 1. Selectbox & Inputs */
            div[data-baseweb="select"] {{
                background-color: rgba(20, 20, 20, 0.9) !important;
                border: 2px solid rgba(0, 217, 255, 0.5) !important;
                border-radius: 8px !important;
            }}
            div[data-baseweb="select"] > div {{ background-color: transparent !important; color: white !important; }}
            div[data-baseweb="select"] svg {{ fill: white !important; }}
            
            /* 2. Dropdowns (Menús desplegables) */
            div[data-baseweb="popover"] {{ background-color: #1E1E1E !important; border: 1px solid #444 !important; }}
            li[role="option"], div[role="option"] {{ background-color: #1E1E1E !important; color: #E0E0E0 !important; }}
            li[role="option"]:hover, div[role="option"]:hover {{ background-color: #333 !important; }}
            
            /* 3. Radio buttons */
            div[role="radiogroup"] > label {{
                background-color: rgba(0, 0, 0, 0.6);
                border: 1px solid rgba(255, 255, 255, 0.3);
                border-radius: 8px;
                padding: 5px 15px;
                margin-right: 10px;
                color: white !important;
            }}
            
            /* 4. Tablas */
            [data-testid="stTableToolbar"], [data-testid="stDataFrameToolbar"], .stDataFrame thead, .stDataFrame th {{
                background-color: #1E1E1E !important; color: white !important;
            }}
            [style*="background-color: white"] {{ background-color: #1E1E1E !important; }}
            p, h3, label {{ color: white !important; }}

            /* --- 5. LINKS DE DESCARGA (Estilo Tags) --- */
            .download-link {{
                text-decoration: none;
                font-weight: bold;
                padding: 4px 10px;
                border-radius: 4px;
                transition: all 0.2s ease;
                margin: 0 5px;
                font-size: 0.9rem;
                display: inline-block;
            }}
            
            /* Estilo específico CSV */
            .csv-tag {{
                color: #00E5FF !important; /* Cyan */
                border: 1px solid rgba(0, 229, 255, 0.3);
                background-color: rgba(0, 229, 255, 0.05);
            }}
            .csv-tag:hover {{
                background-color: rgba(0, 229, 255, 0.2);
                border-color: #00E5FF;
                box-shadow: 0 0 8px rgba(0, 229, 255, 0.4);
                color: white !important;
            }}
            
            /* Estilo específico JSON */
            .json-tag {{
                color: #FF6D00 !important; /* Naranja */
                border: 1px solid rgba(255, 109, 0, 0.3);
                background-color: rgba(255, 109, 0, 0.05);
            }}
            .json-tag:hover {{
                background-color: rgba(255, 109, 0, 0.2);
                border-color: #FF6D00;
                box-shadow: 0 0 8px rgba(255, 109, 0, 0.4);
                color: white !important;
            }}
            
            </style>
            <img id="bg-registros" src="data:image/webp;base64,{img_b64}">
        """), unsafe_allow_html=True)
    
    st.markdown("###  Tabla de Registros")
    
    # ==========================================
    # SELECTOR
    # ==========================================
    opciones_tiempo = {
        "Últimos 50": 50, "Últimos 100": 100, "Últimos 500": 500,
        "Última Hora": timedelta(hours=1), "Últimas 6 Horas": timedelta(hours=6),
        "Últimas 24 Horas": timedelta(hours=24), "Todo el Historial": None
    }
    
    col_filtro1, col_filtro2 = st.columns([2, 1])
    with col_filtro1:
        seleccion = st.radio("Seleccionar Rango:", options=list(opciones_tiempo.keys()), horizontal=True, label_visibility="visible")
    
    valor_seleccion = opciones_tiempo[seleccion]
    
    if valor_seleccion is None:
        df_filtrado = df.copy()
    elif isinstance(valor_seleccion, int):
        df_filtrado = df.sort_values('timestamp', ascending=False).head(valor_seleccion)
    else:
        fecha_corte = df['timestamp'].max() - valor_seleccion
        df_filtrado = df[df['timestamp'] >= fecha_corte].copy()
    
    # Datos
    df_display = df_filtrado[['timestamp', 'dispositivo_id', 'temperatura', 'ph']].copy()
    df_display['timestamp'] = df_display['timestamp'].dt.strftime('%Y-%m-%d %H:%M:%S')
    df_display = df_display.sort_values('timestamp', ascending=False)
    df_display.columns = ['Fecha y Hora', 'Dispositivo', 'Temperatura (°C)', 'pH']
    
    st.info(f" Mostrando **{len(df_display)}** registros ({seleccion})")
    st.dataframe(df_display, use_container_width=True, hide_index=True, height=500)
    
    st.markdown("---")
    
    # SECCIÓN DE DESCARGA: LINKS PUROS (HTML)
    # ==========================================
    
    # 1. Generar los datos en memoria
    csv_str = df_display.to_csv(index=False).encode('utf-8')
    json_str = df_display.to_json(orient='records', indent=2).encode('utf-8')

    # 2. Convertir a Base64 para inyectar en HTML
    b64_csv = base64.b64encode(csv_str).decode()
    b64_json = base64.b64encode(json_str).decode()
    
    # 3. Nombres de archivo dinámicos
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_name_csv = f'registros_{timestamp_str}.csv'
    file_name_json = f'registros_{timestamp_str}.json'
    
    # 4. Renderizar HTML puro (Esto elimina los botones de Streamlit por completo)
    # Usamos clases CSS definidas arriba (.csv-tag y .json-tag)
    st.markdown(f"""
        <div style="text-align: left; font-size: 1rem; color: #cccccc; padding: 10px 0;">
            Puede descargar los registros en formato: 
            <a href="data:text/csv;base64,{b64_csv}" download="{file_name_csv}" class="download-link csv-tag">CSV</a>
            o 
            <a href="data:application/json;base64,{b64_json}" download="{file_name_json}" class="download-link json-tag">JSON</a>
        </div>
    """, unsafe_allow_html=True)