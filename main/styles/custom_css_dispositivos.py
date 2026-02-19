/* =========================================
   ASSETS/STYLES_DISPOSITIVOS.CSS
   Estilos corregidos (Versión "Nuclear" para Inputs)
   ========================================= */

/* --- 1. TARJETAS (EXPANDERS) --- */
/* Forzamos el fondo oscuro en todo el bloque del expander */
div[data-testid="stExpander"] {
    background-color: rgba(20, 20, 20, 0.95) !important; /* Casi negro opaco */
    border: 1px solid rgba(255, 255, 255, 0.1) !important;
    border-radius: 8px !important;
    margin-bottom: 15px;
    color: white !important; /* Texto base blanco */
}

/* Título del Expander (La barra clicable) */
div[data-testid="stExpander"] > details > summary {
    background-color: transparent !important;
    color: white !important; /* Texto blanco forzado */
    border-radius: 8px;
}

div[data-testid="stExpander"] > details > summary:hover {
    color: #00E5FF !important; /* Cyan al pasar mouse */
}

/* Asegurar que cualquier texto dentro del summary (svg, p, span) herede el color */
div[data-testid="stExpander"] > details > summary * {
    color: inherit !important;
    fill: inherit !important;
}

/* Contenido interno del expander */
div[data-testid="stExpander"] > details > div {
    color: white !important;
}

/* --- 2. INPUTS (TEXTO Y NÚMEROS) - CORRECCIÓN VISUAL --- */

/* El contenedor principal del input */
div[data-baseweb="input"] {
    background-color: #2b2b2b !important; /* Gris oscuro sólido */
    border: 1px solid #555 !important;
    border-radius: 5px !important;
}

/* El contenedor interno base */
div[data-baseweb="base-input"] {
    background-color: transparent !important;
}

/* El campo de texto real (donde escribes) */
input.st-ae, input.st-af, input.st-ag, input.st-ah, input[type="text"], input[type="number"] {
    color: white !important;       /* Texto blanco */
    background-color: transparent !important;
    caret-color: #00E5FF !important; /* Cursor cyan */
}

/* Placeholder (Texto gris cuando está vacío) */
input::placeholder {
    color: #888 !important;
}

/* --- 3. BADGES DE ESTADO --- */
.status-badge {
    display: inline-block;
    padding: 3px 10px;
    border-radius: 12px;
    font-size: 0.8rem;
    font-weight: 700;
    margin-left: 10px;
    text-transform: uppercase;
    letter-spacing: 0.5px;
}
.status-online {
    background-color: rgba(0, 255, 0, 0.1); border: 1px solid #00FF00; color: #00FF00;
}
.status-warning {
    background-color: rgba(255, 165, 0, 0.1); border: 1px solid #FFA500; color: #FFA500;
}
.status-offline {
    background-color: rgba(255, 50, 50, 0.1); border: 1px solid #FF4444; color: #FF4444;
}

/* --- 4. TÍTULOS DE SECCIÓN --- */
.section-title {
    color: #00E5FF;
    font-size: 0.85rem;
    font-weight: bold;
    margin-top: 10px;
    margin-bottom: 5px;
    border-bottom: 1px solid rgba(0, 229, 255, 0.3);
    display: block;
}

/* --- 5. BOTÓN GUARDAR --- */
div[data-testid="stFormSubmitButton"] > button {
    width: 100%;
    background-color: rgba(0, 0, 0, 0.3) !important;
    border: 1px solid #00E5FF !important;
    color: #00E5FF !important;
    font-weight: bold !important;
    margin-top: 15px;
}

div[data-testid="stFormSubmitButton"] > button:hover {
    background-color: rgba(0, 229, 255, 0.1) !important;
    box-shadow: 0 0 10px rgba(0, 229, 255, 0.3);
}

div[data-testid="stFormSubmitButton"] > button:focus {
    color: #00E5FF !important;
    border-color: #00E5FF !important;
}