"""Página de Análisis Estadístico (Versión Final: Botón Arreglado)"""
import streamlit as st
import textwrap
import pandas as pd
import numpy as np
import io
import zipfile
import base64
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import timedelta, datetime
from pathlib import Path
from utils import get_image_base64
from styles import apply_analisis_styles

# ==========================================
# 1. CONFIGURACIÓN GLOBAL (SIN GRID)
# ==========================================
plt.style.use('dark_background')

# CAMBIO CLAVE: Usamos style="dark" (sin cuadrícula) explícitamente
sns.set_theme(style="dark", rc={
    "axes.facecolor": "black",
    "figure.facecolor": "none",
    "text.color": "white",
    "xtick.color": "white",
    "ytick.color": "white",
    "axes.labelcolor": "white",
    "axes.edgecolor": "white"
})

def render_analisis_estadistico(df):
    """Renderiza la página de análisis estadístico completa usando Seaborn"""

    # Colores Sólidos para máximo contraste
    COLOR_PH = "#00E5FF"        # Cyan Eléctrico
    COLOR_TEMP = "#FF9100"      # Naranja Intenso
    
    config_map = {
        "pH": {"col": "ph", "c": COLOR_PH, "u": "pH"},
        "Temperatura": {"col": "temperatura", "c": COLOR_TEMP, "u": "°C"}
    }

    # --- VARIABLE DE ESTILO ---
    separador_blanco = "<hr style='border: 0; border-top: 1px solid rgba(255, 255, 255, 0.5); margin: 40px 0;'>"

    # ==========================================
    # 2. FUNCIONES AUXILIARES DE GRÁFICOS
    # ==========================================

    # --- FUNCIÓN DE VIOLÍN (COMENTADA/IGNORADA) ---
    """def crear_grafico_violin_sns(dataframe, columna_dato, titulo, etiqueta_eje, color_hex, mostrar_outliers=False, bg_color='black'):
        if dataframe.empty: return None
        text_color = 'white' if bg_color == 'black' else 'black'
        fig, ax = plt.subplots(figsize=(8, 4), facecolor=bg_color)
        ax.set_facecolor(bg_color)
        ax.grid(False) # Sin grid
        sns.violinplot(
            data=dataframe, x=columna_dato, color=color_hex, 
            inner="box", linewidth=1.5, ax=ax, 
            cut=0 if not mostrar_outliers else 2
        )
        ax.set_title(titulo, fontsize=14, color=text_color, pad=15, fontweight='bold')
        ax.set_xlabel(etiqueta_eje, fontsize=11, color=text_color)
        ax.set_ylabel("")
        sns.despine(left=True, bottom=False)
        plt.tight_layout()
        return fig"""

    def crear_serie_tiempo_sns(dataframe, col_dato, titulo, etiqueta_y, color_hex, bg_color='black'):
        if dataframe.empty: return None
        df_sorted = dataframe.sort_values('timestamp').copy()
        df_sorted['SMA'] = df_sorted[col_dato].rolling(window=5).mean()
        
        text_color = 'white' if bg_color == 'black' else 'black'
        trend_color = 'white' if bg_color == 'black' else 'darkblue'
        
        fig, ax = plt.subplots(figsize=(10, 4), facecolor=bg_color)
        ax.set_facecolor(bg_color)
        
        # ELIMINAR GRID EXPLÍCITAMENTE
        ax.grid(False)
        
        # 1. Valor Real (Sólido alpha=1.0 para que brille)
        sns.lineplot(
            data=df_sorted, x='timestamp', y=col_dato, 
            color=color_hex, 
            alpha=1.0,        
            linewidth=1.5,    
            label='Valor Real', ax=ax
        )
        
        # 2. Tendencia
        sns.lineplot(
            data=df_sorted, x='timestamp', y='SMA', 
            color=trend_color, 
            linewidth=2.5,    
            label='Tendencia', ax=ax
        )
        
        ax.set_title(titulo, fontsize=14, color=text_color, pad=15, fontweight='bold')
        ax.set_ylabel(etiqueta_y, fontsize=11, color=text_color)
        ax.set_xlabel("")
        plt.xticks(rotation=30, ha='right', fontsize=9)
        
        legend_face = 'black' if bg_color == 'black' else 'white'
        legend_text = 'white' if bg_color == 'black' else 'black'
        ax.legend(frameon=True, facecolor=legend_face, edgecolor='#444444', labelcolor=legend_text)
        
        sns.despine()
        plt.tight_layout()
        return fig

    def crear_histograma_sns(dataframe, col_dato, titulo, etiqueta_x, color_hex, n_bins=30, bg_color='black'):
        if dataframe.empty: return None
        media = dataframe[col_dato].mean()
        mediana = dataframe[col_dato].median()
        
        text_color = 'white' if bg_color == 'black' else 'black'
        median_color = '#FF00FF' if bg_color == 'black' else '#9C27B0'
        mean_color = 'white' if bg_color == 'black' else 'darkgreen'
        
        fig, ax = plt.subplots(figsize=(8, 4), facecolor=bg_color)
        ax.set_facecolor(bg_color)
        
        # ELIMINAR GRID EXPLÍCITAMENTE
        ax.grid(False)
        
        sns.histplot(
            data=dataframe, x=col_dato, color=color_hex, kde=True, bins=n_bins, 
            edgecolor=bg_color, 
            linewidth=0.5, 
            alpha=1.0,          
            ax=ax
        )
        
        ax.axvline(media, color=mean_color, linestyle='--', linewidth=1.5, label=f'Media: {media:.2f}')
        ax.axvline(mediana, color=median_color, linestyle=':', linewidth=2, label=f'Mediana: {mediana:.2f}') 
        
        ax.set_title(titulo, fontsize=14, color=text_color, pad=15, fontweight='bold')
        ax.set_xlabel(etiqueta_x, fontsize=11, color=text_color)
        ax.set_ylabel("Frecuencia", fontsize=11, color=text_color)
        
        legend_face = 'black' if bg_color == 'black' else 'white'
        legend_text = 'white' if bg_color == 'black' else 'black'
        legend = ax.legend(frameon=True, facecolor=legend_face, edgecolor='#444444', labelcolor=legend_text)
        plt.setp(legend.get_texts(), color=legend_text)
        
        sns.despine()
        plt.tight_layout()
        return fig

    # ==========================================
    # 2. CARGA DE RECURSOS Y CSS
    # ==========================================
    img_path = Path(__file__).parent.parent / 'assets' / 'alga marina.jpg'
    img_b64 = get_image_base64(img_path) if img_path.exists() else None
    apply_analisis_styles()
    
    if img_b64:
        st.markdown(textwrap.dedent(f"""
            <style>
            #bg-analisis {{
                position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
                z-index: -1; object-fit: cover; opacity: 0.5;
                filter: brightness(1.1) contrast(1.1) saturate(1.3); pointer-events: none;
            }}
            #bg-analisis ~ #bg-analisis {{ display: none !important; }}
            [data-testid="stAppViewContainer"] {{ position: relative; z-index: 1; }}
            
            /* UI General */
            .stMultiSelect > div > div > div, .stSelectbox > div > div > div {{
                background-color: rgba(30, 30, 30, 0.9) !important;
                color: white !important;
                border: 1px solid rgba(255,255,255,0.3) !important;
            }}
            .stMultiSelect div, .stSelectbox div {{ color: white !important; }}
            div[data-baseweb="popover"] {{ background-color: #1E1E1E !important; }}
            label[data-testid="stWidgetLabel"] p {{ color: white !important; }}
            div[role="radiogroup"] > label {{ background-color: rgba(0,0,0,0.5); border: 1px solid rgba(255,255,255,0.2); }}
            
            /* --- CORRECCIÓN BOTÓN GENERAR (CSS ULTRA-AGRESIVO) --- */
            
            /* Selector base más específico */
            div.stButton > button[kind="primary"],
            div.stButton > button[kind="secondary"],
            div.stButton > button,
            button[data-testid="baseButton-secondary"],
            button[data-testid="baseButton-primary"] {{
                background: transparent !important;
                background-color: transparent !important;
                color: #00E5FF !important;
                border: 1px solid rgba(0, 229, 255, 0.5) !important;
                border-radius: 4px !important;
                padding: 0.4rem 1rem !important;
                font-size: 0.9rem !important;
                font-weight: 600 !important;
                transition: all 0.2s ease !important;
                box-shadow: none !important;
            }}
            
            /* Forzar color del texto interno */
            div.stButton > button p,
            div.stButton > button span,
            div.stButton > button div,
            button[data-testid="baseButton-secondary"] p,
            button[data-testid="baseButton-secondary"] span {{
                color: #00E5FF !important;
            }}

            /* Hover State */
            div.stButton > button:hover,
            button[data-testid="baseButton-secondary"]:hover {{
                background-color: rgba(0, 229, 255, 0.15) !important;
                box-shadow: 0 0 12px rgba(0, 229, 255, 0.5) !important;
                border-color: #00E5FF !important;
                color: white !important;
            }}
            
            div.stButton > button:hover p,
            div.stButton > button:hover span,
            button[data-testid="baseButton-secondary"]:hover p {{
                color: white !important;
            }}

            /* Focus State (evitar que se ponga blanco) */
            div.stButton > button:focus,
            div.stButton > button:focus-visible,
            button[data-testid="baseButton-secondary"]:focus {{
                background-color: transparent !important;
                color: #00E5FF !important;
                border-color: #00E5FF !important;
                outline: none !important;
                box-shadow: 0 0 0 2px rgba(0, 229, 255, 0.3) !important;
            }}

            /* Active State */
            div.stButton > button:active,
            button[data-testid="baseButton-secondary"]:active {{
                transform: translateY(1px) !important;
                background-color: rgba(0, 229, 255, 0.25) !important;
            }}

            /* Eliminar cualquier estilo de Streamlit que cause el fondo blanco */
            div.stButton > button::before,
            div.stButton > button::after {{
                display: none !important;
            }}

            /* --- FIN CORRECCIÓN BOTÓN --- */

            /* Botón de Descarga (download_button) */
            div.stDownloadButton > button[kind="primary"],
            div.stDownloadButton > button[kind="secondary"],
            div.stDownloadButton > button {{
                background: transparent !important;
                background-color: transparent !important;
                color: #00FF00 !important;
                border: 1px solid rgba(0, 255, 0, 0.5) !important;
                border-radius: 4px !important;
                padding: 0.4rem 1rem !important;
                font-size: 0.9rem !important;
                font-weight: 600 !important;
                transition: all 0.2s ease !important;
                box-shadow: none !important;
            }}

            div.stDownloadButton > button p,
            div.stDownloadButton > button span {{
                color: #00FF00 !important;
            }}

            div.stDownloadButton > button:hover {{
                background-color: rgba(0, 255, 0, 0.2) !important;
                border-color: #00FF00 !important;
                box-shadow: 0 0 10px rgba(0, 255, 0, 0.4) !important;
                color: white !important;
            }}

            div.stDownloadButton > button:hover p {{
                color: white !important;
            }}

            div.stDownloadButton > button:focus {{
                background-color: transparent !important;
                color: #00FF00 !important;
                border-color: #00FF00 !important;
                outline: none !important;
            }}
            
            </style>
            <img id="bg-analisis" src="data:image/webp;base64,{img_b64}">
        """), unsafe_allow_html=True)

    # 3. INTERFAZ: TÍTULO Y FILTRO
    # ==========================================
    st.markdown(textwrap.dedent("""
        <div style="margin-bottom: 20px;">
            <h2 style='color: white; text-shadow: 0 2px 4px rgba(0,0,0,0.5); margin: 0;'>Estadísticas descriptivas</h2>
            <hr style='width: 100%; border: 0; border-top: 3px solid rgba(255, 255, 255, 0.5); margin-top: 5px; margin-bottom: 15px;'>
        </div>
    """), unsafe_allow_html=True)

    opciones_tiempo = {
        "5 Min": timedelta(minutes=5), "30 Min": timedelta(minutes=30),
        "1 Hora": timedelta(hours=1), "6 Horas": timedelta(hours=6),
        "24 Horas": timedelta(hours=24), "Todo": None
    }
    
    col_filtro, _ = st.columns([2, 1])
    with col_filtro:
        seleccion_tiempo = st.radio("Rango:", options=list(opciones_tiempo.keys()), horizontal=True, label_visibility="collapsed")
    
    delta = opciones_tiempo[seleccion_tiempo]
    if delta:
        fecha_corte = df['timestamp'].max() - delta
        df_filtrado = df[df['timestamp'] >= fecha_corte].copy()
    else:
        df_filtrado = df.copy()

    if df_filtrado.empty:
        st.warning(f" No hay datos registrados en el rango de: {seleccion_tiempo}")
        return

    # ==========================================
    # 4. TABLA
    # ==========================================
    datos_tabla = [
        { "sensor": "pH", "Promedio": f"{df_filtrado['ph'].mean():.2f}", "Mediana": f"{df_filtrado['ph'].median():.2f}", "Minimo": f"{df_filtrado['ph'].min():.2f}", "Maximo": f"{df_filtrado['ph'].max():.2f}" },
        { "sensor": "Temperatura", "Promedio": f"{df_filtrado['temperatura'].mean():.1f}°C", "Mediana": f"{df_filtrado['temperatura'].median():.1f}°C", "Minimo": f"{df_filtrado['temperatura'].min():.1f}°C", "Maximo": f"{df_filtrado['temperatura'].max():.1f}°C" }
    ]
    
    filas_html = ""
    for fila in datos_tabla:
        filas_html += f"""<tr><td>{fila['sensor']}</td><td>{fila['Promedio']}</td><td>{fila['Mediana']}</td><td>{fila['Minimo']}</td><td>{fila['Maximo']}</td></tr>"""

    st.markdown(textwrap.dedent(f"""
        <table class="styled-table">
            <thead><tr><th>Sensor</th><th>Promedio</th><th>Mediana</th><th>Mínimo</th><th>Máximo</th></tr></thead>
            <tbody>{filas_html}</tbody>
        </table>
    """), unsafe_allow_html=True)
    st.markdown(separador_blanco, unsafe_allow_html=True)

    # ==========================================
    # 5. GRÁFICOS (RENDERIZADO CON SEABORN - FONDO NEGRO)
    # ==========================================
    
    # --- Series de Tiempo ---
    st.markdown("### Series de Tiempo (Tendencia)")
    col_ts1, col_ts2 = st.columns(2)
    with col_ts1:
        fig = crear_serie_tiempo_sns(df_filtrado, config_map['pH']['col'], 'Tendencia de pH', config_map['pH']['u'], config_map['pH']['c'])
        st.pyplot(fig)
    with col_ts2:
        fig = crear_serie_tiempo_sns(df_filtrado, config_map['Temperatura']['col'], 'Tendencia de Temperatura', config_map['Temperatura']['u'], config_map['Temperatura']['c'])
        st.pyplot(fig)
    st.markdown(separador_blanco, unsafe_allow_html=True)

    # --- Histogramas ---
    col_hist_titulo, col_hist_ctrl = st.columns([2, 1])
    with col_hist_titulo: st.markdown("### Histogramas de Distribución")
    with col_hist_ctrl: n_bins_selec = st.slider("Número de barras (bins)", 5, 100, 30)

    col_hist1, col_hist2 = st.columns(2)
    with col_hist1:
        fig = crear_histograma_sns(df_filtrado, config_map['pH']['col'], 'Frecuencia de pH', config_map['pH']['u'], config_map['pH']['c'], n_bins=n_bins_selec)
        st.pyplot(fig)
    with col_hist2:
        fig = crear_histograma_sns(df_filtrado, config_map['Temperatura']['col'], 'Frecuencia de Temperatura', config_map['Temperatura']['u'], config_map['Temperatura']['c'], n_bins=n_bins_selec)
        st.pyplot(fig)
    st.markdown(separador_blanco, unsafe_allow_html=True)

    # --- Violines (COMENTADO POR SOLICITUD) ---
    # col_viol_titulo, col_viol_ctrl = st.columns([3, 1])
    # with col_viol_titulo: st.markdown("### Densidad de Datos")
    # with col_viol_ctrl: mostrar_outliers = st.toggle("Ver outliers extremos", value=False)

    # col_box1, col_box2 = st.columns(2)
    # with col_box1:
    #     fig = crear_grafico_violin_sns(df_filtrado, config_map['pH']['col'], 'Densidad de pH', config_map['pH']['u'], config_map['pH']['c'], mostrar_outliers)
    #     st.pyplot(fig)
    # with col_box2:
    #     fig = crear_grafico_violin_sns(df_filtrado, config_map['Temperatura']['col'], 'Densidad de Temperatura', config_map['Temperatura']['u'], config_map['Temperatura']['c'], mostrar_outliers)
    #     st.pyplot(fig)

    # st.markdown(separador_blanco, unsafe_allow_html=True)

    # ==========================================
    # 6. SECCIÓN DE DESCARGA
    # ==========================================
    st.markdown("### Centro de Descargas")
    
    col_dl1, col_dl2, col_dl3 = st.columns(3)
    with col_dl1:
        sensores_sel = st.multiselect("Sensores:", ["pH", "Temperatura"], default=["pH", "Temperatura"])
    with col_dl2:
        # Eliminamos "Densidad" de las opciones por defecto o simplemente lo ignoramos si se selecciona
        graficos_sel = st.multiselect("Gráficos:", ["Tendencia", "Histograma"], default=["Tendencia"])
    with col_dl3:
        estilo_fondo = st.radio("Estilo:", ["Oscuro", "Blanco"])

    if st.button("Generar imagenes"):
        if not sensores_sel or not graficos_sel:
            st.error("❌ Selecciona al menos un sensor y un gráfico.")
        else:
            with st.spinner("Generando imágenes..."):
                try:
                    imagenes_generadas = []
                    
                    # Configurar estilo para descarga
                    if estilo_fondo == "Blanco":
                        plt.style.use('default')
                        # Estilo Blanco (con grid sutil porque en papel se necesita)
                        sns.set_theme(style="whitegrid", rc={
                            "axes.facecolor": "white",
                            "figure.facecolor": "white",
                            "grid.color": "#E5E5E5", 
                            "text.color": "black",
                            "xtick.color": "black",
                            "ytick.color": "black",
                            "axes.labelcolor": "black",
                            "axes.edgecolor": "#333333"
                        })
                        params_save = {'facecolor': 'white'}
                        dl_bg_color = 'white'
                    else:
                        plt.style.use('dark_background')
                        # Estilo Oscuro descarga (SIN GRID TAMBIÉN AQUÍ - style="dark")
                        sns.set_theme(style="dark", rc={
                            "axes.facecolor": "black", 
                            "axes.edgecolor": "white",
                            "xtick.color": "white",
                            "ytick.color": "white",
                            "axes.labelcolor": "white"
                        })
                        params_save = {'facecolor': 'black'}
                        dl_bg_color = 'black'

                    # Generar imágenes
                    for sensor in sensores_sel:
                        cfg = config_map[sensor]
                        
                        if "Tendencia" in graficos_sel:
                            fig = crear_serie_tiempo_sns(df_filtrado, cfg['col'], f"Tendencia {sensor}", cfg['u'], cfg['c'], bg_color=dl_bg_color)
                            if fig:
                                buf = io.BytesIO()
                                fig.savefig(buf, format="png", bbox_inches='tight', dpi=150, **params_save)
                                buf.seek(0)
                                plt.close(fig)
                                imagenes_generadas.append((f"Tendencia_{sensor}.png", buf.read()))
                        
                        if "Histograma" in graficos_sel:
                            fig = crear_histograma_sns(df_filtrado, cfg['col'], f"Histograma {sensor}", cfg['u'], cfg['c'], n_bins=n_bins_selec, bg_color=dl_bg_color)
                            if fig:
                                buf = io.BytesIO()
                                fig.savefig(buf, format="png", bbox_inches='tight', dpi=150, **params_save)
                                buf.seek(0)
                                plt.close(fig)
                                imagenes_generadas.append((f"Histograma_{sensor}.png", buf.read()))
                        
                        # --- VIOLINES DESCARGA (COMENTADO) ---
                        # if "Densidad" in graficos_sel:
                        #     fig = crear_grafico_violin_sns(df_filtrado, cfg['col'], f"Densidad {sensor}", cfg['u'], cfg['c'], mostrar_outliers, bg_color=dl_bg_color)
                        #     if fig:
                        #         buf = io.BytesIO()
                        #         fig.savefig(buf, format="png", bbox_inches='tight', dpi=150, **params_save)
                        #         buf.seek(0)
                        #         plt.close(fig)
                        #         imagenes_generadas.append((f"Densidad_{sensor}.png", buf.read()))

                    # Restaurar estilo UI (Oscuro sin grid)
                    plt.style.use('dark_background')
                    sns.set_theme(style="dark", rc={
                        "axes.facecolor": "black", 
                        "figure.facecolor": "none",
                        "axes.edgecolor": "white",
                        "xtick.color": "white",
                        "ytick.color": "white",
                        "axes.labelcolor": "white"
                    })

                    # Crear ZIP y descargar
                    if imagenes_generadas:
                        zip_buffer = io.BytesIO()
                        with zipfile.ZipFile(zip_buffer, "w") as zf:
                            for nombre, datos in imagenes_generadas:
                                zf.writestr(nombre, datos)
                        
                        st.download_button(
                            label="Descargar",
                            data=zip_buffer.getvalue(),
                            file_name=f"graficos_{datetime.now().strftime('%Y%m%d_%H%M%S')}.zip",
                            mime="application/zip"
                        )
                        st.success(f"{len(imagenes_generadas)} imagen(es) lista(s) para descargar.")

                except Exception as e:
                    st.error(f"Error: {e}")