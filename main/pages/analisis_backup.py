"""Página de Análisis Estadístico (Versión Backup - Plotly)"""
import streamlit as st
import textwrap
import plotly.express as px
import plotly.graph_objects as go
import pandas as pd
import io
import zipfile
import base64
from datetime import timedelta, datetime
from pathlib import Path
from utils import get_image_base64
from styles import apply_analisis_styles

def render_analisis_estadistico(df):
    """Renderiza la página de análisis estadístico completa usando Plotly"""

    # --- VARIABLE DE ESTILO ---
    separador_blanco = "<hr style='border: 0; border-top: 1px solid rgba(255, 255, 255, 0.5); margin: 40px 0;'>"

    # ==========================================
    # 1. FUNCIONES AUXILIARES DE GRÁFICOS (PLOTLY)
    # ==========================================

    def crear_grafico_violin(dataframe, columna_dato, titulo, etiqueta_eje, color_hex, mostrar_todo=False):
        if dataframe.empty: return None
        
        common_axis_config = dict(
            showgrid=True, gridcolor='rgba(255,255,255,0.1)', zeroline=False, 
            tickfont=dict(color='#FFFFFF', size=12), title_font=dict(color='#FFFFFF')
        )

        if not mostrar_todo:
            q_low = dataframe[columna_dato].quantile(0.01)
            q_high = dataframe[columna_dato].quantile(0.99)
            margin = (q_high - q_low) * 0.1
            rango_zoom = [q_low - margin, q_high + margin]
            axis_config = dict(range=rango_zoom, **common_axis_config)
            points_style = False 
        else:
            axis_config = dict(autorange=True, **common_axis_config)
            points_style = "all"

        fig = px.violin(dataframe, x=columna_dato, box=True, points=points_style, orientation='h', title=titulo)
        
        fig.update_layout(
            paper_bgcolor='rgba(0,0,0,0)', plot_bgcolor='rgba(0,0,0,0)',
            title=dict(text=titulo, font=dict(color="#FFFFFF", size=18)),
            margin=dict(l=20, r=20, t=50, b=20),
            xaxis_title=etiqueta_eje, yaxis_title="",
            xaxis=axis_config, yaxis=dict(showticklabels=False),
            height=300,
        )
        fig.update_traces(
            line_color=color_hex, line_width=2, marker=dict(opacity=0.7, size=4, color='white'),
            fillcolor=color_hex, opacity=0.8, side='positive', width=3,
            meanline_visible=True, box_visible=True,
            box_line=dict(color='white', width=2.5), meanline=dict(color='white', width=2.5),
            box_fillcolor='rgba(0,0,0,0.3)' 
        )
        return fig

    def crear_serie_tiempo_sma(dataframe, col_dato, titulo, etiqueta_y, color_hex):
        if dataframe.empty: return None
        
        df_sorted = dataframe.sort_values('timestamp')
        df_sorted['SMA'] = df_sorted[col_dato].rolling(window=5).mean()

        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=df_sorted['timestamp'], y=df_sorted[col_dato],
            mode='lines', name='Valor Real',
            line=dict(color=color_hex, width=1.5), opacity=0.7 
        ))
        fig.add_trace(go.Scatter(
            x=df_sorted['timestamp'], y=df_sorted['SMA'],
            mode='lines', name='Media Móvil',
            line=dict(color='white', width=2.5)
        ))

        fig.update_layout(
            title=dict(text=titulo, font=dict(color="#FFFFFF", size=18)),
            paper_bgcolor='rgba(0,0,0,0)', plot_bgcolor='rgba(0,0,0,0)',
            xaxis=dict(showgrid=False, color='#FFFFFF', tickfont=dict(color='#FFFFFF', size=12), title_font=dict(color='#FFFFFF')),
            yaxis=dict(showgrid=True, gridcolor='rgba(255,255,255,0.1)', color='#FFFFFF', title=etiqueta_y, tickfont=dict(color='#FFFFFF'), title_font=dict(color='#FFFFFF')),
            legend=dict(font=dict(color='#FFFFFF'), orientation="h", y=1.1),
            height=350, margin=dict(l=20, r=20, t=50, b=60), 
        )
        fig.update_xaxes(nticks=10) 
        return fig

    def crear_histograma_distribucion(dataframe, col_dato, titulo, etiqueta_x, color_hex):
        if dataframe.empty: return None

        media = dataframe[col_dato].mean()
        mediana = dataframe[col_dato].median()

        fig = px.histogram(
            dataframe, x=col_dato, nbins=30, title=titulo,
            color_discrete_sequence=[color_hex]
        )

        fig.update_layout(
            paper_bgcolor='rgba(0,0,0,0)', plot_bgcolor='rgba(0,0,0,0)',
            title=dict(text=titulo, font=dict(color="#FFFFFF", size=18)),
            xaxis=dict(showgrid=False, color='#FFFFFF', title=dict(text=etiqueta_x, font=dict(color="white")), tickfont=dict(color='#FFFFFF')),
            yaxis=dict(showgrid=True, gridcolor='rgba(255,255,255,0.1)', color='#FFFFFF', title=dict(text="Frecuencia", font=dict(color="white")), tickfont=dict(color='#FFFFFF')),
            height=350, bargap=0.1, margin=dict(l=20, r=20, t=50, b=20),
        )

        fig.add_vline(x=media, line_width=2, line_dash="dash", line_color="white")
        fig.add_vline(x=mediana, line_width=2, line_dash="dot", line_color="#FF00FF") 

        fig.add_annotation(
            xref="paper", yref="paper", x=0.02, y=0.98,
            text=f"<b>Media:</b> {media:.2f}<br><b>Mediana:</b> {mediana:.2f}",
            showarrow=False, font=dict(color="white", size=11), align="left",
            bgcolor="rgba(0,0,0,0.6)", bordercolor="rgba(255,255,255,0.3)", borderwidth=1
        )
        return fig

    # ==========================================
    # 2. CARGA DE RECURSOS (Imagen de fondo)
    # ==========================================
    img_path = Path(__file__).parent.parent / 'assets' / 'fondo hexagonal negro.jpg'
    img_b64 = get_image_base64(img_path) if img_path.exists() else None

    if img_b64:
        st.markdown(textwrap.dedent(f"""
            <style>
            #bg-analisis {{
                position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
                z-index: -1; object-fit: cover; opacity: 0.5;
            }}
            #bg-analisis ~ #bg-analisis {{ display: none !important; }}
            [data-testid="stAppViewContainer"] {{ position: relative; z-index: 1; }}
            
            /* --- ESTILOS GENERALES --- */
            .stMultiSelect > div > div > div, .stSelectbox > div > div > div {{
                background-color: rgba(30, 30, 30, 0.9) !important;
                color: white !important;
                border: 1px solid rgba(255,255,255,0.3) !important;
            }}
            .stMultiSelect div, .stSelectbox div {{ color: white !important; }}
            
            label[data-testid="stWidgetLabel"] p {{ color: white !important; }}
            div[role="radiogroup"] > label {{ 
                background-color: rgba(0,0,0,0.5); 
                border: 1px solid rgba(255,255,255,0.2); 
                border-radius: 10px; padding: 5px 15px; margin-right: 10px; color: white !important; 
            }}
            
            /* Botones */
            div.stButton > button {{
                background-color: rgba(255, 255, 255, 0.1) !important;
                color: #E0E0E0 !important;
                border: 1px solid rgba(255, 255, 255, 0.25) !important;
            }}
            </style>
            <img id="bg-analisis" src="data:image/webp;base64,{img_b64}">
        """), unsafe_allow_html=True)

    # ==========================================
    # 3. INTERFAZ: TÍTULO Y FILTRO
    # ==========================================
    st.markdown(textwrap.dedent("""
        <div style="margin-bottom: 20px;">
            <h2 style='color: white; text-shadow: 0 2px 4px rgba(0,0,0,0.5); margin: 0;'>Estadísticas descriptivas (Legacy Plotly)</h2>
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
        tiempo_sel = st.selectbox("📅 Seleccionar intervalo de tiempo:", list(opciones_tiempo.keys()), index=2)

    # Lógica de filtrado
    if opciones_tiempo[tiempo_sel]:
        fecha_limite = df['timestamp'].max() - opciones_tiempo[tiempo_sel]
        df_filtrado = df[df['timestamp'] >= fecha_limite].copy()
    else:
        df_filtrado = df.copy()

    # ==========================================
    # 4. TARJETAS DE MÉTRICAS (KPIs)
    # ==========================================
    if df_filtrado.empty:
        st.warning("No hay datos para el rango seleccionado.")
        return

    col1, col2 = st.columns(2)
    
    # KPIs pH
    ph_mean, ph_std = df_filtrado['ph'].mean(), df_filtrado['ph'].std()
    ph_min, ph_max = df_filtrado['ph'].min(), df_filtrado['ph'].max()
    
    with col1:
        st.markdown(f"""
            <div style="background-color: rgba(0, 229, 255, 0.1); border: 1px solid rgba(0, 229, 255, 0.3); border-radius: 10px; padding: 15px; margin-bottom: 10px;">
                <h3 style="color: #00E5FF; margin: 0 0 10px 0;">💧 pH del Agua</h3>
                <div style="display: flex; justify-content: space-between;">
                    <div><span style="color: #aaa; font-size: 0.9em;">Promedio</span><br><span style="font-size: 1.5em; font-weight: bold; color: white;">{ph_mean:.2f}</span></div>
                    <div><span style="color: #aaa; font-size: 0.9em;">Desviación</span><br><span style="font-size: 1.2em; color: #ddd;">±{ph_std:.3f}</span></div>
                </div>
                <div style="margin-top: 10px; padding-top: 10px; border-top: 1px solid rgba(255,255,255,0.1); font-size: 0.9em; color: #ccc;">
                    Rango: <span style="color: white; font-weight: bold;">{ph_min:.2f}</span> - <span style="color: white; font-weight: bold;">{ph_max:.2f}</span>
                </div>
            </div>
        """, unsafe_allow_html=True)

    # KPIs Temperatura
    temp_mean, temp_std = df_filtrado['temperatura'].mean(), df_filtrado['temperatura'].std()
    temp_min, temp_max = df_filtrado['temperatura'].min(), df_filtrado['temperatura'].max()

    with col2:
        st.markdown(f"""
            <div style="background-color: rgba(255, 109, 0, 0.1); border: 1px solid rgba(255, 109, 0, 0.3); border-radius: 10px; padding: 15px; margin-bottom: 10px;">
                <h3 style="color: #FF6D00; margin: 0 0 10px 0;">🌡️ Temperatura</h3>
                <div style="display: flex; justify-content: space-between;">
                    <div><span style="color: #aaa; font-size: 0.9em;">Promedio</span><br><span style="font-size: 1.5em; font-weight: bold; color: white;">{temp_mean:.1f}°C</span></div>
                    <div><span style="color: #aaa; font-size: 0.9em;">Desviación</span><br><span style="font-size: 1.2em; color: #ddd;">±{temp_std:.2f}°C</span></div>
                </div>
                <div style="margin-top: 10px; padding-top: 10px; border-top: 1px solid rgba(255,255,255,0.1); font-size: 0.9em; color: #ccc;">
                    Rango: <span style="color: white; font-weight: bold;">{temp_min:.1f}°C</span> - <span style="color: white; font-weight: bold;">{temp_max:.1f}°C</span>
                </div>
            </div>
        """, unsafe_allow_html=True)

    st.markdown(separador_blanco, unsafe_allow_html=True)

    # ==========================================
    # 5. GRÁFICOS
    # ==========================================
    
    st.markdown("### Series de Tiempo (Tendencia)")
    col_ts1, col_ts2 = st.columns(2)
    with col_ts1:
        fig_ts_ph = crear_serie_tiempo_sma(df_filtrado, 'ph', 'Tendencia de pH', 'pH', '#00E5FF')
        st.plotly_chart(fig_ts_ph, use_container_width=True)
    with col_ts2:
        fig_ts_temp = crear_serie_tiempo_sma(df_filtrado, 'temperatura', 'Tendencia de Temperatura', '°C', '#FF6D00')
        st.plotly_chart(fig_ts_temp, use_container_width=True)
    st.markdown(separador_blanco, unsafe_allow_html=True)

    st.markdown("### Histogramas de Distribución")
    col_hist1, col_hist2 = st.columns(2)
    with col_hist1:
        fig_hist_ph = crear_histograma_distribucion(df_filtrado, 'ph', 'Frecuencia de pH', 'pH', '#00E5FF')
        st.plotly_chart(fig_hist_ph, use_container_width=True)
    with col_hist2:
        fig_hist_temp = crear_histograma_distribucion(df_filtrado, 'temperatura', 'Frecuencia de Temperatura', '°C', '#FF6D00')
        st.plotly_chart(fig_hist_temp, use_container_width=True)
    st.markdown(separador_blanco, unsafe_allow_html=True)

    col_titulo_viol, col_toggle = st.columns([3, 1])
    with col_titulo_viol:
         st.markdown(f"### Densidad de Datos")
    with col_toggle:
        mostrar_outliers = st.toggle("Ver outliers extremos", value=False)
    col_box1, col_box2 = st.columns(2)
    with col_box1:
        fig_violin_ph = crear_grafico_violin(df_filtrado, 'ph', 'Densidad de pH', 'pH', '#00E5FF', mostrar_todo=mostrar_outliers)
        st.plotly_chart(fig_violin_ph, use_container_width=True)
    with col_box2:
        fig_violin_temp = crear_grafico_violin(df_filtrado, 'temperatura', 'Densidad de Temperatura', '°C', '#FF6D00', mostrar_todo=mostrar_outliers)
        st.plotly_chart(fig_violin_temp, use_container_width=True)

    st.markdown(separador_blanco, unsafe_allow_html=True)

    # ==========================================
    # 6. SECCIÓN DE DESCARGA ORIGINAL (Plotly estático)
    # ==========================================
    st.markdown("### 📥 Centro de Descargas")
    
    col_dl1, col_dl2, col_dl3 = st.columns(3)
    with col_dl1:
        sensores_sel = st.multiselect("Sensores:", ["pH", "Temperatura"], default=["pH", "Temperatura"])
    with col_dl2:
        graficos_sel = st.multiselect("Gráficos:", ["Tendencia", "Histograma", "Densidad"], default=["Tendencia"])
    with col_dl3:
        estilo_fondo = st.radio("Estilo de Fondo:", ["Oscuro (Presentación)", "Blanco (Informe)"])

    if st.button("📸 Generar Imágenes"):
        if not sensores_sel or not graficos_sel:
            st.error("Selecciona al menos un sensor y un gráfico.")
        else:
            with st.spinner("Procesando..."):
                try:
                    imagenes_generadas = []
                    config_map = {"pH": {"col": "ph", "c": "#00E5FF", "u": "pH"}, "Temperatura": {"col": "temperatura", "c": "#FF6D00", "u": "°C"}}

                    # Configurar estilo visual para la imagen exportada
                    if estilo_fondo == "Blanco (Informe)":
                        bg_color, font_color, grid_color = "white", "black", "rgba(0,0,0,0.1)"
                    else:
                        bg_color, font_color, grid_color = "black", "white", "rgba(255,255,255,0.1)"

                    for sensor in sensores_sel:
                        cfg = config_map[sensor]
                        def procesar_y_guardar(fig, nombre):
                            if fig:
                                fig.update_layout(plot_bgcolor=bg_color, paper_bgcolor=bg_color, font_color=font_color, xaxis=dict(color=font_color, gridcolor=grid_color), yaxis=dict(color=font_color, gridcolor=grid_color), title_font=dict(color=font_color), legend=dict(font=dict(color=font_color)))
                                fig.update_xaxes(tickfont=dict(color=font_color), title_font=dict(color=font_color))
                                fig.update_yaxes(tickfont=dict(color=font_color), title_font=dict(color=font_color))
                                img_bytes = fig.to_image(format="png", width=1200, height=600, scale=2)
                                imagenes_generadas.append((nombre, img_bytes))

                        if "Tendencia" in graficos_sel:
                            fig = crear_serie_tiempo_sma(df_filtrado, cfg['col'], f"Tendencia {sensor}", cfg['u'], cfg['c'])
                            procesar_y_guardar(fig, f"Tendencia_{sensor}.png")
                        if "Histograma" in graficos_sel:
                            fig = crear_histograma_distribucion(df_filtrado, cfg['col'], f"Histograma {sensor}", cfg['u'], cfg['c'])
                            procesar_y_guardar(fig, f"Histograma_{sensor}.png")
                        if "Densidad" in graficos_sel:
                            fig = crear_grafico_violin(df_filtrado, cfg['col'], f"Densidad {sensor}", cfg['u'], cfg['c'], mostrar_todo=mostrar_outliers)
                            procesar_y_guardar(fig, f"Densidad_{sensor}.png")

                    if len(imagenes_generadas) == 1:
                        n, d = imagenes_generadas[0]
                        st.download_button(f"⬇️ Descargar {n}", d, n, "image/png")
                    elif len(imagenes_generadas) > 1:
                        zip_buffer = io.BytesIO()
                        with zipfile.ZipFile(zip_buffer, "w") as zf:
                            for n, d in imagenes_generadas: zf.writestr(n, d)
                        st.download_button(f"⬇️ Descargar todo (ZIP)", zip_buffer.getvalue(), "reporte.zip", "application/zip")
                    
                    if imagenes_generadas: st.success("¡Imágenes listas!")

                except Exception as e:
                    st.error(f"Error: {e}")
