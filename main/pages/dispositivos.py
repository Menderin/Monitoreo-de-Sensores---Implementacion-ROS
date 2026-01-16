"""Página de Información de Dispositivos"""
import streamlit as st
from config.settings import Settings

def render_dispositivos(df):
    """Renderiza la página de información de dispositivos"""
    
    st.markdown("### 🔧 Información de Dispositivos")
    
    col_dev1, col_dev2 = st.columns(2)
    
    with col_dev1:
        st.markdown("#### 🌊 Sensor de pH")
        st.info("""
        **Modelo:** Sensor pH Analógico  
        **Rango:** 0 - 14 pH  
        **Precisión:** ±0.1 pH  
        **Estado:** ✅ Operativo
        """)
        
        st.metric("Última Lectura", f"{df['ph'].iloc[-1]:.2f} pH")
        st.metric("Total de Lecturas", len(df))
        
    with col_dev2:
        st.markdown("#### 🌡️ Sensor de Temperatura")
        st.info("""
        **Modelo:** DHT22 / DS18B20  
        **Rango:** -40°C a 80°C  
        **Precisión:** ±0.5°C  
        **Estado:** ✅ Operativo
        """)
        
        st.metric("Última Lectura", f"{df['temperatura'].iloc[-1]:.1f}°C")
        st.metric("Total de Lecturas", len(df))
    
    st.markdown("---")
    
    st.markdown("#### 📡 Conexión y Base de Datos")
    col_info1, col_info2, col_info3 = st.columns(3)
    
    with col_info1:
        st.metric("Base de Datos", Settings.MONGO_DB)
    with col_info2:
        st.metric("Colección", Settings.MONGO_COLLECTION)
    with col_info3:
        st.metric("Estado Conexión", "✅ Conectado")
