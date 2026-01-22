"""Estilos CSS personalizados para el dashboard"""
import streamlit as st

def apply_custom_styles():
    """Aplica todos los estilos CSS personalizados"""
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

        /* Ocultar header para evitar espacio superior */
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
        
        /* Fondo simple oscuro */
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
        
        /* Títulos - forzar texto blanco */
        h1, h2, h3, h4, h5, h6 {
            color: #FFFFFF !important;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        
        .main-header h1, .main-header p {
            color: #FFFFFF !important;
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
        
        /* Forzar texto blanco en todo el documento */
        p, div, span, label, .stMarkdown {
            color: #FFFFFF !important;
        }
        
        /* Estilo de pestañas como barra de navegación */
        .stTabs {
            margin-top: 0 !important;
        }
        .stTabs [data-baseweb="tab-list"] {
            gap: 8px;
            background: rgba(255, 255, 255, 0.25);
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
        
        /* Cambiar el color del indicador de tab de rojo a verde */
        .stTabs [data-baseweb="tab-highlight"] {
            background-color: #00ff88 !important;
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
            background: rgba(0, 217, 255, 0.1);
            border-radius: 15px;
            margin-bottom: 15px;
            border: 1px solid rgba(0, 217, 255, 0.3);
        }
        
        .main-header h1 {
            font-size: 2rem;
            margin-bottom: 5px;
            color: #FFFFFF !important;
        }
        
        .main-header p {
            font-size: 1rem;
            color: #FFFFFF !important;
            margin: 0;
        }
        </style>
    """, unsafe_allow_html=True)

def apply_tab_styles():
    """Aplica estilos personalizados para las pestañas (tabs)"""
    st.markdown("""
        <style>
        /* 1. Contenedor de las pestañas */
        [data-testid="stTabs"] button {
            flex-grow: 1 !important;
            width: 100% !important;
            display: flex !important;
            justify-content: center !important;
        }

        /* 2. Texto dentro de las pestañas */
        [data-testid="stTabs"] button div[data-testid="stMarkdownContainer"] p {
            font-size: 1.5rem !important;
            font-weight: normal !important;
            padding: 10px 0px !important;
        }
        
        /* 3. PESTAÑA SELECCIONADA */
        [data-testid="stTabs"] button[aria-selected="true"] {
             /* Fondo Degradado */
             background: linear-gradient(90deg, #00d4ff 0%, #0060ff 100%) !important;
             color: white !important;
             
             /* Eliminar bordes estándar */
             border: none !important;
             outline: none !important;
             box-shadow: none !important;
        }
        
        /* 4. Pestañas NO seleccionadas */
        [data-testid="stTabs"] button[aria-selected="false"] {
            background-color: rgba(255, 255, 255, 0.1) !important;
            border: none !important;
            box-shadow: none !important;
        }
        </style>
    """, unsafe_allow_html=True)
    
def apply_metric_styles():
    """Aplica estilos personalizados para las métricas"""
    st.markdown("""
        <style>
        /* Centrar el contenedor principal de la métrica */
        [data-testid="stMetric"] {
            display: flex;
            flex-direction: column;
            align-items: center !important;
            text-align: center !important;
        }

        /* Ajustar el valor de la métrica */
        [data-testid="stMetricValue"] {
            font-size: 3rem !important;
            font-weight: normal !important;
        }

        /* Ajustar y centrar la etiqueta */
        [data-testid="stMetricLabel"] {
            font-size: 2.8rem !important;
            width: 100%;
            justify-content: center !important;
        }

        /* Ajustar el Delta */
        [data-testid="stMetricDelta"] {
            font-size: 1.8rem !important;
            font-weight: normal !important;
            justify-content: center !important;
        }
        </style>
    """, unsafe_allow_html=True)

def apply_chart_styles():
    """Aplica estilos personalizados para los gráficos de Plotly"""
    st.markdown("""
        <style>
        [data-testid="stPlotlyChart"] {
            background-color: rgba(0, 0, 0, 0.5);
            backdrop-filter: blur(10px);
            -webkit-backdrop-filter: blur(10px);
            border-radius: 15px;
            padding: 15px;
            border: 1px solid rgba(255, 255, 255, 0.1);
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
            margin-top: 20px;
        }
        </style>
    """, unsafe_allow_html=True)

def apply_analisis_styles():
    """Aplica estilos específicos para la página de análisis estadístico (Tablas y Gráficos)"""
    import textwrap
    
    css = """
        <style>
        /* --- 2. ESTILOS DE LA TABLA --- */
        .styled-table {
            width: 100%;
            border-collapse: collapse;
            margin: 25px 0;
            font-size: 1.1rem;
            font-family: sans-serif;
            border-radius: 15px;
            overflow: hidden;
            box-shadow: 0 0 20px rgba(0, 0, 0, 0.15);
            background-color: rgba(0, 0, 0, 0.6);
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255, 255, 255, 0.1);
        }
        
        .styled-table thead tr {
            background-color: rgba(0, 212, 255, 0.2);
            color: #ffffff;
            text-align: center;
            font-weight: bold;
        }
        
        .styled-table th, .styled-table td {
            padding: 12px 15px;
            text-align: center;
            color: white;
            border-bottom: 1px solid rgba(255, 255, 255, 0.1);
        }
        
        /* Nombre del sensor a la izquierda y en color */
        .styled-table tbody td:first-child {
            text-align: left;
            font-weight: bold;
            color: #00d4ff;
        }

        /* --- 3. ELIMINAR FONDO BLANCO DE CONTENEDORES MARKDOWN --- */
        [data-testid="stMarkdownContainer"] {
            background-color: transparent !important;
        }
        
        .stMarkdown {
            background-color: transparent !important;
        }
        
        /* Hacer transparentes TODOS los elementos de bloque */
        [data-testid="stVerticalBlock"] > div {
            background-color: transparent !important;
        }
        
        /* Eliminar TODOS los fondos blancos posibles */
        .element-container {
            background-color: transparent !important;
        }
        
        [data-testid="stHorizontalBlock"] {
            background-color: transparent !important;
        }
        
        [data-testid="column"] {
            background-color: transparent !important;
        }
        
        /* Forzar transparencia en TODOS los divs dentro del contenedor principal */
        .main .block-container > div {
            background-color: transparent !important;
        }
        
        /* Eliminar cualquier padding/margin que pueda causar la caja blanca */
        .main .block-container [style*="background"] {
            background: transparent !important;
        }

        /* --- 4. ESTILOS DE GRÁFICOS --- */
        [data-testid="stPlotlyChart"] {
            background-color: rgba(0, 0, 0, 0.5);
            backdrop-filter: blur(10px);
            border-radius: 15px;
            padding: 15px;
            border: 1px solid rgba(255, 255, 255, 0.1);
            margin-top: 20px;
        }
        </style>
    """
    st.markdown(textwrap.dedent(css), unsafe_allow_html=True)
