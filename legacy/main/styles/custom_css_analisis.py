/* =========================================
   CUSTOM_CSS_ANALISIS.CSS
   Estilos corregidos para la página de Análisis
   ========================================= */

/* --- 1. FONDO DE PÁGINA --- */
#bg-analisis {
    position: fixed !important;
    top: 0 !important;
    left: 0 !important;
    width: 100vw !important;
    height: 100vh !important;
    z-index: -1 !important;
    object-fit: cover !important;
    opacity: 0.5 !important; /* AJUSTA ESTE VALOR: 0.1 (muy tenue) a 1.0 (opaco) */
    filter: brightness(0.9) contrast(1.1) saturate(1.2) !important;
    pointer-events: none !important;
}

/* Evitar duplicados del fondo */
img[id="bg-analisis"] ~ img[id="bg-analisis"] {
    display: none !important;
}

/* Asegurar que el contenido esté por encima */
[data-testid="stAppViewContainer"] {
    position: relative !important;
    z-index: 1 !important;
}

/* Fondo semi-transparente para contenido principal - MÁS CLARO */
[data-testid="stAppViewContainer"] > div:first-child {
    background-color: rgba(10, 20, 30, 0.4) !important; /* BAJÉ DE 0.7 A 0.4 */
    backdrop-filter: blur(3px) !important;
}

/* --- 2. SELECTORES Y CONTROLES --- */

/* Labels generales */
label[data-testid="stWidgetLabel"],
label[data-testid="stWidgetLabel"] p,
label[data-testid="stWidgetLabel"] span {
    color: #E0E0E0 !important;
    font-weight: 500 !important;
}

/* Selectbox - Contenedor principal */
div[data-testid="stSelectbox"] > div > div {
    background-color: rgba(30, 30, 30, 0.95) !important;
    border: 1px solid rgba(0, 229, 255, 0.4) !important;
    border-radius: 6px !important;
}

/* Selectbox cuando está abierto/activo */
div[data-testid="stSelectbox"] > div > div:focus-within {
    border-color: #00E5FF !important;
    box-shadow: 0 0 0 1px #00E5FF !important;
}

/* Texto seleccionado en el selectbox */
div[data-testid="stSelectbox"] input,
div[data-testid="stSelectbox"] div[data-baseweb="select"] > div,
div[data-testid="stSelectbox"] div[data-baseweb="select"] span {
    color: #FFFFFF !important;
    background-color: transparent !important;
}

/* Dropdown del selectbox (menú desplegable) */
div[data-baseweb="popover"] {
    background-color: rgba(20, 25, 30, 0.98) !important;
    border: 1px solid rgba(0, 229, 255, 0.3) !important;
    border-radius: 6px !important;
    backdrop-filter: blur(10px) !important;
}

/* CRÍTICO: Eliminar cualquier fondo blanco del contenedor interno */
div[data-baseweb="popover"] > div,
div[data-baseweb="popover"] [role="listbox"],
div[data-baseweb="popover"] [role="menu"] {
    background-color: transparent !important;
}

/* Opciones dentro del dropdown */
div[data-baseweb="popover"] ul {
    background-color: transparent !important;
}

div[data-baseweb="popover"] li,
div[role="option"],
div[data-baseweb="menu-item"] {
    color: #E0E0E0 !important;
    background-color: transparent !important; /* COMPLETAMENTE TRANSPARENTE */
    padding: 10px 16px !important;
    transition: all 0.2s ease !important;
    border-left: 3px solid transparent !important;
    border-right: 3px solid transparent !important;
}

/* Forzar que los divs internos también sean transparentes */
div[role="option"] > div,
div[data-baseweb="menu-item"] > div {
    background-color: transparent !important;
}

div[data-baseweb="popover"] li:hover,
div[role="option"]:hover,
div[data-baseweb="menu-item"]:hover {
    background-color: transparent !important;
    color: #FFFFFF !important;
    border-left-color: #00E5FF !important;
}

/* Forzar hover en elementos internos */
div[role="option"]:hover > div,
div[data-baseweb="menu-item"]:hover > div {
    background-color: transparent !important;
}

/* Opción seleccionada en el dropdown */
div[data-baseweb="popover"] li[aria-selected="true"],
div[role="option"][aria-selected="true"],
div[data-baseweb="menu-item"][aria-selected="true"] {
    background-color: transparent !important;
    color: #FFFFFF !important;
    font-weight: 600 !important;
    border-left-color: #00E5FF !important;
    border-right-color: #00E5FF !important;
}

/* Forzar transparencia en elementos internos seleccionados */
div[role="option"][aria-selected="true"] > div,
div[data-baseweb="menu-item"][aria-selected="true"] > div {
    background-color: transparent !important;
}

/* Iconos y emojis en las opciones */
div[data-baseweb="popover"] li span,
div[role="option"] span {
    color: inherit !important;
}

/* Multiselect */
div[data-testid="stMultiSelect"] > div > div {
    background-color: rgba(30, 30, 30, 0.95) !important;
    border: 1px solid rgba(255, 255, 255, 0.3) !important;
    border-radius: 4px !important;
}

div[data-testid="stMultiSelect"] input,
div[data-testid="stMultiSelect"] span {
    color: #FFFFFF !important;
}

/* Tags seleccionados en multiselect */
div[data-testid="stMultiSelect"] span[data-baseweb="tag"] {
    background-color: rgba(0, 229, 255, 0.3) !important;
    border: 1px solid #00E5FF !important;
    color: #FFFFFF !important;
}

/* --- 3. RADIO BUTTONS (Rango Temporal) --- */
div[role="radiogroup"] {
    display: flex !important;
    gap: 8px !important;
    flex-wrap: wrap !important;
}

div[role="radiogroup"] > label {
    background-color: rgba(20, 20, 20, 0.8) !important;
    border: 1px solid rgba(255, 255, 255, 0.25) !important;
    border-radius: 6px !important;
    padding: 8px 16px !important;
    color: #E0E0E0 !important;
    transition: all 0.2s ease !important;
    cursor: pointer !important;
}

div[role="radiogroup"] > label:hover {
    background-color: rgba(40, 40, 40, 0.9) !important;
    border-color: #00E5FF !important;
    transform: translateY(-1px) !important;
}

/* Radio button seleccionado */
div[role="radiogroup"] > label[data-checked="true"],
div[role="radiogroup"] > label:has(input:checked) {
    background-color: rgba(0, 229, 255, 0.25) !important;
    border-color: #00E5FF !important;
    color: #00E5FF !important;
    font-weight: 600 !important;
    box-shadow: 0 0 10px rgba(0, 229, 255, 0.3) !important;
}

/* Ocultar el círculo del radio */
div[role="radiogroup"] input[type="radio"] {
    display: none !important;
}

/* --- 4. SLIDER --- */
div[data-testid="stSlider"] label {
    color: #E0E0E0 !important;
}

div[data-testid="stSlider"] div[role="slider"] {
    background-color: #00E5FF !important;
}

div[data-testid="stSlider"] div[data-baseweb="slider"] > div {
    background-color: rgba(255, 255, 255, 0.2) !important;
}

/* --- 5. BOTÓN "GENERAR" (st.button - COMPACTO) --- */
div.stButton > button {
    background: linear-gradient(135deg, rgba(0, 229, 255, 0.1), rgba(0, 229, 255, 0.05)) !important;
    color: #00E5FF !important;
    border: 2px solid #00E5FF !important;
    border-radius: 6px !important;
    padding: 6px 16px !important; /* MÁS COMPACTO */
    font-weight: 600 !important;
    font-size: 0.85rem !important; /* MÁS PEQUEÑO */
    text-transform: uppercase !important;
    letter-spacing: 0.3px !important;
    transition: all 0.3s ease !important;
    box-shadow: 0 2px 8px rgba(0, 229, 255, 0.2) !important;
    max-width: 200px !important; /* LÍMITE DE ANCHO */
}

div.stButton > button p,
div.stButton > button span {
    color: #00E5FF !important;
    margin: 0 !important;
    font-size: 0.85rem !important; /* AJUSTADO */
    line-height: 1.2 !important; /* COMPACTO */
}

div.stButton > button:hover {
    background: linear-gradient(135deg, rgba(0, 229, 255, 0.25), rgba(0, 229, 255, 0.15)) !important;
    border-color: #00FFFF !important;
    box-shadow: 0 0 20px rgba(0, 229, 255, 0.5) !important;
    transform: translateY(-2px) !important;
}

div.stButton > button:hover p,
div.stButton > button:hover span {
    color: #FFFFFF !important;
}

div.stButton > button:active,
div.stButton > button:focus {
    background: rgba(0, 229, 255, 0.2) !important;
    border-color: #00E5FF !important;
    outline: none !important;
    box-shadow: 0 0 15px rgba(0, 229, 255, 0.6) !important;
}

/* --- 6. BOTÓN "DESCARGAR" (st.download_button - COMPACTO) --- */
[data-testid="stDownloadButton"] button {
    background: linear-gradient(135deg, rgba(0, 255, 0, 0.1), rgba(0, 255, 0, 0.05)) !important;
    color: #00FF00 !important;
    border: 2px solid #00FF00 !important;
    border-radius: 6px !important;
    padding: 6px 16px !important; /* MÁS COMPACTO */
    font-weight: 600 !important;
    font-size: 0.85rem !important; /* MÁS PEQUEÑO */
    text-transform: uppercase !important;
    letter-spacing: 0.3px !important;
    transition: all 0.3s ease !important;
    box-shadow: 0 2px 8px rgba(0, 255, 0, 0.2) !important;
    max-width: 180px !important; /* LÍMITE DE ANCHO */
}

[data-testid="stDownloadButton"] button p,
[data-testid="stDownloadButton"] button span {
    color: #00FF00 !important;
    margin: 0 !important;
    font-size: 0.85rem !important; /* AJUSTADO */
    line-height: 1.2 !important; /* COMPACTO */
}

[data-testid="stDownloadButton"] button:hover {
    background: linear-gradient(135deg, rgba(0, 255, 0, 0.25), rgba(0, 255, 0, 0.15)) !important;
    border-color: #00FF66 !important;
    box-shadow: 0 0 20px rgba(0, 255, 0, 0.5) !important;
    transform: translateY(-2px) !important;
}

[data-testid="stDownloadButton"] button:hover p,
[data-testid="stDownloadButton"] button:hover span {
    color: #FFFFFF !important;
}

[data-testid="stDownloadButton"] button:active,
[data-testid="stDownloadButton"] button:focus,
[data-testid="stDownloadButton"] button:focus-visible {
    background: rgba(0, 255, 0, 0.2) !important;
    border-color: #00FF00 !important;
    outline: none !important;
    box-shadow: 0 0 15px rgba(0, 255, 0, 0.6) !important;
}

/* --- 7. TABLA ESTILIZADA --- */
.styled-table {
    width: 100% !important;
    border-collapse: collapse !important;
    margin: 20px 0 !important;
    font-size: 0.95rem !important;
    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif !important;
    min-width: 400px !important;
    background-color: rgba(20, 20, 20, 0.95) !important;
    border-radius: 8px !important;
    overflow: hidden !important;
    box-shadow: 0 4px 20px rgba(0, 0, 0, 0.5) !important;
    color: #FFFFFF !important;
}

.styled-table thead tr {
    background: linear-gradient(135deg, rgba(0, 229, 255, 0.3), rgba(0, 229, 255, 0.2)) !important;
    color: #FFFFFF !important;
    text-align: left !important;
    font-weight: bold !important;
    text-transform: uppercase !important;
    letter-spacing: 0.5px !important;
    border-bottom: 2px solid #00E5FF !important;
}

.styled-table th {
    padding: 16px 20px !important;
    color: #FFFFFF !important;
    font-size: 0.9rem !important;
}

.styled-table td {
    padding: 14px 20px !important;
    color: #E0E0E0 !important;
}

.styled-table tbody tr {
    border-bottom: 1px solid rgba(255, 255, 255, 0.1) !important;
    transition: all 0.2s ease !important;
}

.styled-table tbody tr:last-of-type {
    border-bottom: 2px solid #00E5FF !important;
}

.styled-table tbody tr:hover {
    background-color: rgba(0, 229, 255, 0.08) !important;
    transform: scale(1.01) !important;
    box-shadow: 0 2px 8px rgba(0, 229, 255, 0.2) !important;
}

/* Primera columna (nombres de sensores) en negrita */
.styled-table tbody td:first-child {
    font-weight: 600 !important;
    color: #00E5FF !important;
}

/* --- 8. TÍTULOS Y SEPARADORES --- */
h2, h3 {
    color: #FFFFFF !important;
    text-shadow: 0 2px 4px rgba(0, 0, 0, 0.5) !important;
}

hr {
    border: 0 !important;
    border-top: 2px solid rgba(0, 229, 255, 0.3) !important;
    margin: 30px 0 !important;
}

/* --- 9. MENSAJES DE ESTADO --- */
div[data-testid="stSuccess"],
div[data-testid="stWarning"],
div[data-testid="stError"],
div[data-testid="stInfo"] {
    background-color: rgba(30, 30, 30, 0.95) !important;
    border-radius: 6px !important;
    padding: 12px !important;
    color: #FFFFFF !important;
}

/* --- 10. SPINNER DE CARGA --- */
div[data-testid="stSpinner"] > div {
    border-color: #00E5FF transparent transparent transparent !important;
}

/* --- 11. COLUMNAS --- */
div[data-testid="column"] {
    padding: 8px !important;
}

/* --- 12. MARKDOWN PERSONALIZADO --- */
.stMarkdown p {
    color: #E0E0E0 !important;
}

.stMarkdown strong {
    color: #00E5FF !important;
}