/* =========================================
   Estilos específicos para la página de Registros
   ========================================= */

/* --- 1. FONDO Y CONTENEDOR PRINCIPAL --- */
#bg-registros {
    position: fixed;
    top: 0;
    left: 0;
    width: 100vw;
    height: 100vh;
    z-index: -1;
    object-fit: cover;
    opacity: 0.9 !important;
    pointer-events: none;
}

/* Evitar duplicados */
#bg-registros ~ #bg-registros { display: none !important; }

/* CORRECCIÓN FUNDAMENTAL: 
   1. El nombre correcto es stAppViewContainer (sin el 1).
   2. background-color: transparent es vital para ver la imagen de fondo. 
*/
[data-testid="stAppViewContainer"] {
    position: relative;
    z-index: 1;
    background-color: transparent !important;
}

/* Opcional: Hacer transparente el header para que se vea el fondo arriba */
[data-testid="stHeader"] {
    background-color: rgba(0,0,0,0) !important;
}

/* --- 2. UI GENERAL --- */

/* Selectbox & Inputs */
div[data-baseweb="select"] {
    background-color: rgba(20, 20, 20, 0.9) !important;
    border: 2px solid rgba(0, 217, 255, 0.5) !important;
    border-radius: 8px !important;
}
div[data-baseweb="select"] > div {
    background-color: transparent !important;
    color: white !important;
}
div[data-baseweb="select"] svg {
    fill: white !important;
}

/* Dropdowns */
div[data-baseweb="popover"] {
    background-color: #1E1E1E !important;
    border: 1px solid #444 !important;
}
li[role="option"], div[role="option"] {
    background-color: #1E1E1E !important;
    color: #E0E0E0 !important;
}
li[role="option"]:hover, div[role="option"]:hover {
    background-color: #333 !important;
}

/* Radio buttons */
div[role="radiogroup"] > label {
    background-color: rgba(0, 0, 0, 0.7);
    border: 1px solid rgba(255, 255, 255, 0.3);
    border-radius: 8px;
    padding: 5px 15px;
    margin-right: 10px;
    color: white !important;
    transition: all 0.2s;
}
div[role="radiogroup"] > label:hover {
    background-color: rgba(0, 229, 255, 0.1);
    border-color: #00E5FF;
}

/* --- 3. ESTILOS DE LA TABLA --- */
/* Forzamos colores oscuros en el contenedor de la tabla */
[data-testid="stDataFrame"] {
    background-color: rgba(20, 20, 20, 0.8) !important;
    border: 1px solid rgba(255, 255, 255, 0.1);
    border-radius: 5px;
    padding: 5px;
}

/* --- 4. LINKS DE DESCARGA (Tags) --- */
.download-link {
    text-decoration: none;
    font-weight: bold;
    padding: 4px 10px;
    border-radius: 4px;
    transition: all 0.2s ease;
    margin: 0 5px;
    font-size: 0.9rem;
    display: inline-block;
}

/* CSV - Cyan */
.csv-tag {
    color: #00E5FF !important;
    border: 1px solid rgba(0, 229, 255, 0.3);
    background-color: rgba(0, 229, 255, 0.05);
}
.csv-tag:hover {
    background-color: rgba(0, 229, 255, 0.2);
    border-color: #00E5FF;
    box-shadow: 0 0 8px rgba(0, 229, 255, 0.4);
    color: white !important;
}

/* JSON - Naranja */
.json-tag {
    color: #FF6D00 !important;
    border: 1px solid rgba(255, 109, 0, 0.3);
    background-color: rgba(255, 109, 0, 0.05);
}
.json-tag:hover {
    background-color: rgba(255, 109, 0, 0.2);
    border-color: #FF6D00;
    box-shadow: 0 0 8px rgba(255, 109, 0, 0.4);
    color: white !important;
}