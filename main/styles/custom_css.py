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
