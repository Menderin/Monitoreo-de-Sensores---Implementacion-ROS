"""Página de Análisis Estadístico"""
import streamlit as st
import textwrap  # <--- IMPORTANTE
from pathlib import Path
from utils import get_image_base64
from components import crear_grafico_caja
from styles import apply_analisis_styles

def render_analisis_estadistico(df):
    """Renderiza la página de análisis estadístico con tabla personalizada"""

    # 1. Imagen de fondo
    img_path = Path(__file__).parent.parent / 'assets' / 'alga marina.jpg'
    img_b64 = get_image_base64(img_path) if img_path.exists() else None
    
    # 2. Aplicar estilos estáticos (tablas, gráficos, etc.)
    apply_analisis_styles()
    
    if img_b64:
        st.markdown(textwrap.dedent(f"""
            <style>
            /* --- 1. CONFIGURACIÓN DEL FONDO --- */
            #bg-analisis {{
                position: fixed;
                top: 0;
                left: 0;
                width: 100vw;
                height: 100vh;
                z-index: -1;
                object-fit: cover;
                opacity: 0.5;
                filter: brightness(1.1) contrast(1.1) saturate(1.3);
                pointer-events: none;
            }}
            
            #bg-analisis ~ #bg-analisis {{ display: none !important; }}
            
            [data-testid="stAppViewContainer"] {{
                position: relative;
                z-index: 1;
            }}
            </style>
            <img id="bg-analisis" src="data:image/webp;base64,{img_b64}">
        """), unsafe_allow_html=True)
    
    # --- TÍTULO PRINCIPAL (CORREGIDO) ---
    # Usamos textwrap.dedent para limpiar los espacios que crean el cuadro blanco
    st.markdown(textwrap.dedent("""
        <div style="display: flex; flex-direction: column; align-items: flex-start; width: 100%; margin-bottom: 20px;">
            <h2 style='color: white; text-shadow: 0 2px 4px rgba(0,0,0,0.5); margin: 0; padding: 0; text-align: left !important;'>
                Estadísticas descriptivas
            </h2>
            <hr style='width: 100%; border: 0; border-top: 3px solid rgba(255, 255, 255, 0.5); margin-top: 5px; margin-bottom: 15px;'>
            
            
        </div>
    """), unsafe_allow_html=True)
    
    # --- PREPARACIÓN DE DATOS ---
    # Usamos tus nombres en Español.
    # Nota: Las claves aquí (ej: "Promedio") deben ser IDENTICAS a las del bucle de abajo.
    datos_tabla = [
        {
            "sensor": "Ph",
            "Promedio": f"{df['ph'].mean():.2f}",
            "Mediana": f"{df['ph'].median():.2f}",
            # "std": f"{df['ph'].std():.2f}",  <-- COMENTADO (No se mostrará)
            "Minimo": f"{df['ph'].min():.2f}",
            "Maximo": f"{df['ph'].max():.2f}",
        },
        {
            "sensor": "Temperatura",
            "Promedio": f"{df['temperatura'].mean():.1f}°C",
            "Mediana": f"{df['temperatura'].median():.1f}°C",
            # "std": f"{df['temperatura'].std():.1f}°C", <-- COMENTADO
            "Minimo": f"{df['temperatura'].min():.1f}°C",
            "Maximo": f"{df['temperatura'].max():.1f}°C",
        }
    ]
    
    # --- CONSTRUCCIÓN DE LA TABLA HTML ---
    filas_html = ""
    for fila in datos_tabla:
        filas_html += f"""
        <tr>
            <td>{fila['sensor']}</td>
            <td>{fila['Promedio']}</td>
            <td>{fila['Mediana']}</td>
            <td>{fila['Minimo']}</td>
            <td>{fila['Maximo']}</td>
        </tr>"""

    # Ensamblamos la tabla (Quitamos también el encabezado "Desv. Est.")
    tabla_completa = textwrap.dedent(f"""
        <table class="styled-table">
            <thead>
                <tr>
                    <th>Sensor</th>
                    <th>Promedio</th>
                    <th>Mediana</th>
                    <th>Mínimo</th>
                    <th>Máximo</th>
                </tr>
            </thead>
            <tbody>
                {filas_html}
            </tbody>
        </table>
    """)
    
    # Renderizar la tabla
    st.markdown(tabla_completa, unsafe_allow_html=True)
    
    st.markdown("---")
    
    # --- SECCIÓN: Gráficos de caja ---
    st.markdown("### 📦 Distribución de Datos (Box Plots)")
    col_box1, col_box2 = st.columns(2)
    
    with col_box1:
        fig_box_ph = crear_grafico_caja(
            df, 'ph',
            'Distribución de pH',
            'ph',
            'pH'
        )
        st.plotly_chart(fig_box_ph, use_container_width=True)
    
    with col_box2:
        fig_box_temp = crear_grafico_caja(
            df, 'temperatura',
            'Distribución de Temperatura',
            'temperatura',
            'Temperatura (°C)'
        )
        st.plotly_chart(fig_box_temp, use_container_width=True)