"""Página de Análisis Estadístico (Versión Final: Histogramas con Alto Contraste)"""
import streamlit as st
import textwrap
import pandas as pd
import numpy as np
import io
import zipfile
import base64
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.dates as mdates
from datetime import timedelta, datetime, timezone
import dateutil.parser
from pathlib import Path
from utils import get_image_base64 
from database.mongo_handler import MongoHandler, obtener_dispositivos 
from config.settings import Settings
from styles import apply_analisis_styles

def load_css_file(css_file_path):
    """Carga un archivo CSS externo."""
    with open(css_file_path) as f:
        st.markdown(f'<style>{f.read()}</style>', unsafe_allow_html=True)

# Configuración Gráfica Base
plt.style.use('dark_background')

def get_device_status_emoji(ultima_conexion):
    if not ultima_conexion: return "🔴"
    try:
        ultima = dateutil.parser.isoparse(str(ultima_conexion))
        if ultima.tzinfo is None: ultima = ultima.replace(tzinfo=timezone.utc)
        ahora = datetime.now(timezone.utc)
        diff_min = (ahora - ultima).total_seconds() / 60
        return "🟢" if diff_min < 10 else "🔴"
    except: return "🔴"

@st.cache_data(ttl=300, show_spinner=False)
def cargar_historial_completo():
    try:
        client = MongoHandler.get_client()
        if client is None: return pd.DataFrame()
        db = client[Settings.MONGO_DB]; collection = db[Settings.MONGO_COLLECTION]
        
        projection = {
            '_id': 0, 'timestamp': 1, 'dispositivo_id': 1, 
            'datos.temperatura': 1, 'datos.ph': 1
        }
        
        datos = list(collection.find({}, projection).sort('timestamp', -1))
        
        if not datos: return pd.DataFrame()
        
        registros = []
        for d in datos:
            try:
                temp = d.get('datos', {}).get('temperatura')
                ph = d.get('datos', {}).get('ph')
                dev_id = d.get('dispositivo_id', 'Desconocido')
                
                if temp is not None and ph is not None:
                    ts = pd.to_datetime(d['timestamp'])
                    if ts.tzinfo is None: ts = ts.tz_localize('UTC')
                    ts = ts.tz_convert('America/Santiago')
                    
                    registros.append({
                        'timestamp': ts, 'dispositivo_id': dev_id, 
                        'temperatura': temp, 'ph': ph
                    })
            except: continue
            
        return pd.DataFrame(registros).sort_values('timestamp')
    except Exception as e:
        st.error(f"Error cargando historial: {e}"); return pd.DataFrame()

def render_analisis_estadistico(df_inicial):
    css_path = Path(__file__).parent.parent / 'styles' / 'custom_css_analisis.py'
    if css_path.exists(): load_css_file(css_path)
    
    img_path = Path(__file__).parent.parent / 'assets' / 'alga marina.jpg'
    img_b64 = get_image_base64(img_path) if img_path.exists() else None
    if img_b64: st.markdown(f'<img id="bg-analisis" src="data:image/webp;base64,{img_b64}">', unsafe_allow_html=True)

    # Definimos mapa base (se actualiza dinámicamente)
    config_map = {"pH": {"col": "ph"}, "Temperatura": {"col": "temperatura"}}
    
    # --- FUNCIONES DE GRÁFICOS ---
    def crear_serie_tiempo_sns(dataframe, col_dato, titulo_base, etiqueta_y, color_hex, rango_seleccion, subtexto_intervalo, bg_color):
        if dataframe.empty: return None
        df_sorted = dataframe.sort_values('timestamp').copy()
        
        n_datos = len(df_sorted)
        ventana = 5 if n_datos < 1000 else 20
        df_sorted['SMA'] = df_sorted[col_dato].rolling(window=ventana, min_periods=1).mean()
        
        es_oscuro = bg_color in ['black', '#000000', '#111111', '#1a1a1a']
        text_color = 'white' if es_oscuro else '#222222'
        trend_color = 'white' if es_oscuro else '#1A237E' 
        grid_color = '#444' if es_oscuro else '#CCCCCC'
        
        fig, ax = plt.subplots(figsize=(10, 4), facecolor=bg_color)
        ax.set_facecolor(bg_color)
        ax.grid(True, linestyle=':', alpha=0.5, color=grid_color)
        
        sns.lineplot(data=df_sorted, x='timestamp', y=col_dato, color=color_hex, alpha=0.9, linewidth=1.5, label='Valor Real', ax=ax)
        sns.lineplot(data=df_sorted, x='timestamp', y='SMA', color=trend_color, linewidth=2.5, label='Tendencia', ax=ax)
        ax.set_xlim(df_sorted['timestamp'].min(), df_sorted['timestamp'].max())
        
        from datetime import timezone, timedelta as td
        chile_tz = timezone(td(hours=-3))
        if rango_seleccion == "5 Min": formatter = mdates.DateFormatter('%H:%M:%S', tz=chile_tz)
        elif rango_seleccion in ["30 Min", "1 Hora", "6 Horas"]: formatter = mdates.DateFormatter('%H:%M', tz=chile_tz)
        elif rango_seleccion == "24 Horas": formatter = mdates.DateFormatter('%H:%M', tz=chile_tz); ax.xaxis.set_major_locator(mdates.HourLocator(interval=3))
        else: formatter = mdates.DateFormatter('%d/%m %H:%M', tz=chile_tz); ax.xaxis.set_major_locator(mdates.AutoDateLocator())

        ax.xaxis.set_major_formatter(formatter)
        plt.setp(ax.xaxis.get_majorticklabels(), rotation=30, ha='right', fontsize=9, color=text_color)
        ax.tick_params(axis='y', colors=text_color)
        for spine in ax.spines.values(): spine.set_edgecolor(text_color)
        
        ax.set_title(f"{titulo_base} ({subtexto_intervalo})", fontsize=14, color=text_color, pad=15, fontweight='normal', fontstyle='italic')
        ax.set_ylabel(etiqueta_y, fontsize=12, color=text_color); ax.set_xlabel("")
        
        legend = ax.legend(frameon=True, facecolor=bg_color, edgecolor=text_color)
        for text in legend.get_texts(): text.set_color(text_color)
            
        sns.despine(left=False, bottom=False); plt.tight_layout()
        return fig

    # --- HISTOGRAMA MODIFICADO (Acepta color de línea separado) ---
    def crear_histograma_sns(dataframe, col_dato, titulo_base, etiqueta_x, color_hex, line_color_hex, n_bins, subtexto_intervalo, bg_color):
        """
        Histograma con curva KDE en color controlado manualmente
        """
        if dataframe.empty: 
            return None
        
        es_oscuro = bg_color in ['black', '#000000', '#111111', '#1a1a1a']
        text_color = 'white' if es_oscuro else '#222222'
        mean_line_color = 'white' if es_oscuro else '#D50000'

        fig, ax = plt.subplots(figsize=(10, 4), facecolor=bg_color)
        ax.set_facecolor(bg_color)
        ax.grid(False)
        
        # MÉTODO 1: Histograma + KDE por separado
        # Primero el histograma sin KDE
        n, bins_edges, patches = ax.hist(
            dataframe[col_dato].dropna(),
            bins=n_bins,
            color=color_hex,
            alpha=0.9,
            edgecolor=bg_color,
            linewidth=0.5,
            density=False  # Frecuencia absoluta
        )
        
        # Ahora la curva KDE manualmente
        from scipy import stats
        import numpy as np
        
        datos = dataframe[col_dato].dropna()
        kde = stats.gaussian_kde(datos)
        
        x_min, x_max = datos.min(), datos.max()
        x_range = np.linspace(x_min, x_max, 300)
        density = kde(x_range)
        
        # Escalar KDE para que se vea bien con el histograma
        # La densidad se multiplica por el total de datos y ancho de bin
        bin_width = (x_max - x_min) / n_bins
        scaled_density = density * len(datos) * bin_width
        
        # Dibujar la curva KDE con el color deseado
        ax.plot(x_range, scaled_density, color=line_color_hex, linewidth=2.5, label='Densidad')
        
        # Línea de media
        ax.axvline(
            dataframe[col_dato].mean(), 
            color=mean_line_color, 
            linestyle='--', 
            linewidth=2,
            label=f'Media: {dataframe[col_dato].mean():.2f}'
        )
        
        # Estilos
        ax.tick_params(axis='x', colors=text_color)
        ax.tick_params(axis='y', colors=text_color)
        for spine in ax.spines.values(): 
            spine.set_edgecolor(text_color)
        
        ax.set_title(
            f"{titulo_base} ({subtexto_intervalo})", 
            fontsize=12, 
            color=text_color, 
            pad=15, 
            fontweight='normal', 
            fontstyle='italic'
        )
        ax.set_xlabel(etiqueta_x, fontsize=12, color=text_color)
        ax.set_ylabel("Frecuencia", fontsize=12, color=text_color)
        
        # Leyenda
        legend = ax.legend(frameon=True, facecolor=bg_color, edgecolor=text_color, loc='upper left')
        for text in legend.get_texts(): 
            text.set_color(text_color)
            
        plt.tight_layout()
        return fig

    st.markdown("""<div style="margin-bottom: 20px;"><h2 style='color: white; text-shadow: 0 2px 4px rgba(0,0,0,0.5); margin: 0;'>Estadísticas descriptivas</h2><hr style='width: 100%; border: 0; border-top: 3px solid rgba(255, 255, 255, 0.5); margin-top: 5px; margin-bottom: 15px;'></div>""", unsafe_allow_html=True)

    # --- FILTROS ---
    lista_db_dispositivos = obtener_dispositivos()
    opciones_dispositivos = {"TODOS": "📡 Ver todos los dispositivos (Combinado)"}
    lista_db_dispositivos.sort(key=lambda x: x.get('conexion', {}).get('ultima', ''), reverse=True)
    for dev in lista_db_dispositivos:
        dev_id = dev.get('_id'); nombre = dev.get('nombre', dev_id)
        emoji = get_device_status_emoji(dev.get('conexion', {}).get('ultima'))
        opciones_dispositivos[dev_id] = f"{emoji} {nombre} ({dev_id})"

    col_dev, col_time = st.columns([1, 2])
    with col_dev:
        st.markdown("<label style='font-size: 0.8rem; color: #ccc;'>Seleccionar Dispositivo:</label>", unsafe_allow_html=True)
        seleccion_ui = st.selectbox("Sel_Disp_Hidden", options=list(opciones_dispositivos.keys()), format_func=lambda x: opciones_dispositivos[x], label_visibility="collapsed")
    with col_time:
        st.markdown("<label style='font-size: 0.8rem; color: #ccc;'>Rango Temporal:</label>", unsafe_allow_html=True)
        opciones_tiempo = {"5 Min": timedelta(minutes=5), "30 Min": timedelta(minutes=30), "1 Hora": timedelta(hours=1), "6 Horas": timedelta(hours=6), "24 Horas": timedelta(hours=24), "Todo": None}
        sel_tiempo = st.radio("Rango:", options=list(opciones_tiempo.keys()), horizontal=True, label_visibility="collapsed")
    
    delta = opciones_tiempo[sel_tiempo]
    needs_full_load = (sel_tiempo in ["24 Horas", "Todo"]) or (seleccion_ui != "TODOS")

    if needs_full_load:
        with st.spinner("Cargando historial..."): df_base = cargar_historial_completo()
    else: df_base = df_inicial.copy()

    if df_base.empty: st.warning("No hay datos."); return
    if seleccion_ui != "TODOS":
        if 'dispositivo_id' in df_base.columns: df_filtrado_dev = df_base[df_base['dispositivo_id'] == seleccion_ui].copy()
        else: st.error("Error ID."); return
    else: df_filtrado_dev = df_base.copy()

    if df_filtrado_dev.empty: st.warning(f"Sin registros."); return
    if delta:
        fecha_min = df_filtrado_dev['timestamp'].max() - delta
        df_filtrado = df_filtrado_dev[df_filtrado_dev['timestamp'] >= fecha_min].copy()
    else: df_filtrado = df_filtrado_dev.copy()

    if df_filtrado.empty: st.warning(f"Sin datos en rango."); return

    t_min = df_filtrado['timestamp'].min(); t_max = df_filtrado['timestamp'].max()
    intervalo_str = f"{t_min.strftime('%d/%m %H:%M')} - {t_max.strftime('%H:%M')}"
    filename_suffix = f"{t_min.strftime('%d%m_%H%M')}_{seleccion_ui}"

    filas = ""
    for s, cfg in config_map.items():
        col = cfg['col']
        if col in df_filtrado.columns:
            filas += f"<tr><td>{s}</td><td>{df_filtrado[col].mean():.2f}</td><td>{df_filtrado[col].median():.2f}</td><td>{df_filtrado[col].min():.2f}</td><td>{df_filtrado[col].max():.2f}</td></tr>"
    st.markdown(f"""<table class="styled-table"><thead><tr><th>Sensor</th><th>Promedio</th><th>Mediana</th><th>Mínimo</th><th>Máximo</th></tr></thead><tbody>{filas}</tbody></table>""", unsafe_allow_html=True)
    st.markdown("<hr style='border: 0; border-top: 1px solid rgba(255, 255, 255, 0.5); margin: 40px 0;'>", unsafe_allow_html=True)

    # --- TEMA VISUAL ---
    paletas = {"Claro (Hueso)": "#F2F0E6", "Oscuro (Negro)": "black", "Blanco Puro": "white"}
    col_tema, _ = st.columns([1, 2])
    with col_tema:
        st.markdown("<label style='font-size: 0.8rem; color: #ccc;'>Tema Visual:</label>", unsafe_allow_html=True)
        seleccion_tema = st.radio("Tema_Graficos", options=list(paletas.keys()), index=0, horizontal=True, label_visibility="collapsed")
    
    bg_color_elegido = paletas[seleccion_tema]

    # --- DEFINICIÓN DE COLORES (REFINADO PARA CONTRASTE) ---
    if seleccion_tema == "Oscuro (Negro)":
        # NEÓN (Fondo Oscuro) - La línea es blanca para resaltar
        c_ph_fill = "#00E5FF"   # Cyan
        c_ph_line = "white"     # Línea Blanca
        c_temp_fill = "#FF9100" # Naranja
        c_temp_line = "white"   # Línea Blanca
    else:
        c_ph_fill = "#1DC7E0"   # color barras ph
        c_ph_line = "#266BEB"   # color linea
        c_temp_fill = "#CC5437" # color barra temp
        c_temp_line = "#FAB120" # color linea

    # --- GRÁFICOS ---
    st.markdown("### Series de Tiempo")
    c1, c2 = st.columns(2)
    with c1: st.pyplot(crear_serie_tiempo_sns(df_filtrado, 'ph', 'Tendencia pH', 'pH', c_ph_fill, sel_tiempo, intervalo_str, bg_color=bg_color_elegido))
    with c2: st.pyplot(crear_serie_tiempo_sns(df_filtrado, 'temperatura', 'Tendencia Temp', '°C', c_temp_fill, sel_tiempo, intervalo_str, bg_color=bg_color_elegido))
    
    st.markdown("### Distribución")
    bins = st.slider("Bins", 5, 100, 30)
    c3, c4 = st.columns(2)
    
    # Pasamos FILL y LINE por separado
    with c3: st.pyplot(crear_histograma_sns(df_filtrado, 'ph', 'Distribución pH', 'pH', 
                       color_hex=c_ph_fill, line_color_hex=c_ph_line, 
                       n_bins=bins, subtexto_intervalo=intervalo_str, bg_color=bg_color_elegido))
    
    with c4: st.pyplot(crear_histograma_sns(df_filtrado, 'temperatura', 'Distribución Temp', '°C', 
                       color_hex=c_temp_fill, line_color_hex=c_temp_line, 
                       n_bins=bins, subtexto_intervalo=intervalo_str, bg_color=bg_color_elegido))

    # --- DESCARGA ---
    st.markdown("### Centro de Descargas")
    cd1, cd2, cd3 = st.columns(3)
    with cd1: sens = st.multiselect("Datos:", ["pH", "Temperatura"], default=["pH", "Temperatura"])
    with cd2: grafs = st.multiselect("Gráficos:", ["Tendencia", "Histograma"], default=["Tendencia"])
    with cd3: estilo_descarga = st.radio("Estilo Descarga:", ["Igual a Vista", "Oscuro"], index=0) #para volver la opcion de blanco agregar esto despues de "Oscuro":, "Blanco"

    if st.button("Generar imágenes"):
        if not sens or not grafs: st.error("Selecciona opciones.")
        else:
            with st.spinner("Procesando..."):
                try:
                    imgs = []
                    # Configurar colores descarga
                    if estilo_descarga == "Igual a Vista":
                        bg_dl = bg_color_elegido
                        dl_ph_fill, dl_ph_line = c_ph_fill, c_ph_line
                        dl_temp_fill, dl_temp_line = c_temp_fill, c_temp_line
                    elif estilo_descarga == "Oscuro":
                        bg_dl = "black"
                        dl_ph_fill, dl_ph_line = "#00E5FF", "white"
                        dl_temp_fill, dl_temp_line = "#FF9100", "white"
                    #else: # Blanco
                     #   bg_dl = "white"
                      #  dl_ph_fill, dl_ph_line = "#00796B", "#004D40"
                      #  dl_temp_fill, dl_temp_line = "#D84315", "#B71C1C"

                    # Mapa temporal para el loop
                    mapa_dl = {
                        "pH": {"col": "ph", "c": dl_ph_fill, "line": dl_ph_line, "u": "pH"},
                        "Temperatura": {"col": "temperatura", "c": dl_temp_fill, "line": dl_temp_line, "u": "°C"}
                    }
                    
                    with plt.style.context('default'):
                        for s in sens:
                            cfg = mapa_dl[s]
                            if "Tendencia" in grafs:
                                fig = crear_serie_tiempo_sns(df_filtrado, cfg['col'], f"Tendencia {s}", cfg['u'], cfg['c'], sel_tiempo, intervalo_str, bg_color=bg_dl)
                                buf = io.BytesIO(); fig.savefig(buf, format="png", bbox_inches='tight', dpi=300, facecolor=bg_dl); buf.seek(0); plt.close(fig)
                                imgs.append((f"Tendencia_{s}_{filename_suffix}.png", buf.read()))
                            if "Histograma" in grafs:
                                fig = crear_histograma_sns(df_filtrado, cfg['col'], f"Histograma {s}", cfg['u'], cfg['c'], cfg['line'], bins, intervalo_str, bg_color=bg_dl)
                                buf = io.BytesIO(); fig.savefig(buf, format="png", bbox_inches='tight', dpi=300, facecolor=bg_dl); buf.seek(0); plt.close(fig)
                                imgs.append((f"Histograma_{s}_{filename_suffix}.png", buf.read()))
                    
                    if imgs:
                        zip_buf = io.BytesIO()
                        with zipfile.ZipFile(zip_buf, "w") as z:
                            for n, d in imgs: z.writestr(n, d)
                        zip_name = f"reporte_{filename_suffix}.zip"
                        st.download_button(label="Descargar ZIP", data=zip_buf.getvalue(), file_name=zip_name, mime="application/zip")
                        st.success(f"Imágenes generadas: {zip_name}")
                except Exception as e: st.error(f"Error: {e}")