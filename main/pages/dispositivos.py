"""Página de Gestión de Dispositivos"""
import streamlit as st
import pandas as pd
from pathlib import Path
from datetime import datetime, timezone
import dateutil.parser
from utils import get_image_base64
from database.mongo_handler import obtener_dispositivos, actualizar_dispositivo

def load_css_file(css_file_path):
    with open(css_file_path) as f:
        st.markdown(f'<style>{f.read()}</style>', unsafe_allow_html=True)

def calcular_estado(ultima_conexion_str):
    """
    Determina el estado (Online/Offline) basado en el tiempo 
    transcurrido desde la última conexión registrada.
    """
    if not ultima_conexion_str:
        return "NUEVO / SIN DATOS", "status-offline"
    
    try:
        # Parseamos la fecha ISO almacenada en Mongo
        ultima = dateutil.parser.isoparse(str(ultima_conexion_str))
        
        # Aseguramos zona horaria UTC para comparar correctamente
        if ultima.tzinfo is None:
            ultima = ultima.replace(tzinfo=timezone.utc)
            
        ahora = datetime.now(timezone.utc)
        diferencia_minutos = (ahora - ultima).total_seconds() / 60
        
        if diferencia_minutos < 5:
            return "🟢 ONLINE", "status-online"
        elif diferencia_minutos < 60:
            return f"⚠️ INACTIVO ({int(diferencia_minutos)}m)", "status-warning"
        else:
            # Si lleva más de una hora, mostramos horas
            horas = int(diferencia_minutos / 60)
            return f"🔴 OFFLINE ({horas}h)", "status-offline"
            
    except Exception:
        return "ERROR FECHA", "status-offline"

def render_dispositivos():
    css_path = Path(__file__).parent.parent / 'styles' / 'custom_css_dispositivos.py'
    if css_path.exists(): load_css_file(css_path)
    
    img_path = Path(__file__).parent.parent / 'assets' / 'fondo hexagonal blanco.jpg'
    img_b64 = get_image_base64(img_path) if img_path.exists() else None
    if img_b64:
        st.markdown(f'<img id="bg-registros" src="data:image/webp;base64,{img_b64}">', unsafe_allow_html=True)

    # 2. Encabezado
    st.markdown("###  Mis Dispositivos (ESP32)")
    st.markdown("""
        <div style="color: #bbb; font-size: 0.9rem; margin-bottom: 20px;">
            Administra tus nodos, verifica su estado de conexión y configura los rangos de alerta
            para el monitoreo.
        </div>
    """, unsafe_allow_html=True)
    
    # 3. Obtener lista de dispositivos desde BD
    dispositivos = obtener_dispositivos()
    
    if not dispositivos:
        st.warning("No se encontraron dispositivos registrados en el sistema.")
        return

    # 4. Renderizar tarjetas
    for dev in dispositivos:
        # Extraer datos básicos
        dev_id = dev.get('_id', 'Desconocido')
        nombre_actual = dev.get('nombre', dev_id)
        
        # Calcular estado de conexión
        texto_estado, clase_css = calcular_estado(dev.get('conexion', {}).get('ultima'))
        
        # Título del Expander (Nombre + ID)
        titulo_expander = f"{nombre_actual} | ID: {dev_id}"
        
        with st.expander(titulo_expander, expanded=False):
            
            # --- Encabezado de la Tarjeta (Estado) ---
            st.markdown(f"""
                <div style="display: flex; align-items: center; margin-bottom: 20px; padding-bottom: 10px; border-bottom: 1px solid #333;">
                    <span style="color: #fff; font-weight: bold;">Estado Actual:</span>
                    <span class="status-badge {clase_css}">{texto_estado}</span>
                </div>
            """, unsafe_allow_html=True)
            
            # --- Formulario de Configuración ---
            with st.form(key=f"form_{dev_id}"):
                col_identidad, col_umbrales = st.columns(2)
                
                # Columna 1: Identidad (Nombre)
                with col_identidad:
                    st.markdown('<p class="section-title">🏷️ IDENTIDAD</p>', unsafe_allow_html=True)
                    nuevo_nombre = st.text_input(
                        "Nombre del Dispositivo", 
                        value=nombre_actual,
                        help="Un nombre fácil de recordar (ej. 'Piscina 1')"
                    )
                
                # Columna 2: Umbrales de Alerta (Configuración)
                with col_umbrales:
                    st.markdown('<p class="section-title">🔔 UMBRALES DE ALERTA</p>', unsafe_allow_html=True)
                    
                    # Recuperar configuración existente o usar defaults
                    umbrales = dev.get('umbrales', {})
                    
                    c1, c2, c3 = st.columns(3)
                    with c1:
                        min_ph = st.number_input("Min pH", value=float(umbrales.get('ph_min', 6.0)), step=0.1, key=f"min_{dev_id}")
                    with c2:
                        max_ph = st.number_input("Max pH", value=float(umbrales.get('ph_max', 8.5)), step=0.1, key=f"max_{dev_id}")
                    with c3:
                        max_temp = st.number_input("Max Temp °C", value=float(umbrales.get('temp_max', 30.0)), step=0.5, key=f"temp_{dev_id}")
                    
                    st.caption("Si los valores recibidos salen de este rango, el sistema marcará alerta.")

                # Botón de Guardar
                st.write("") # Espacio
                if st.form_submit_button("Guardar Configuración", use_container_width=True):
                    
                    # Datos a actualizar en MongoDB
                    update_data = {
                        "nombre": nuevo_nombre,
                        "umbrales": {
                            "ph_min": min_ph,
                            "ph_max": max_ph,
                            "temp_max": max_temp
                        }
                    }
                    
                    if actualizar_dispositivo(dev_id, update_data):
                        st.success(f" Dispositivo '{nuevo_nombre}' actualizado correctamente.")
                        st.rerun() # Recargar para ver el cambio de nombre arriba
                    else:
                        st.error("❌Error al guardar los cambios.")