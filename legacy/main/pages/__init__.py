"""Módulo de páginas del dashboard"""
from .monitoreo_vivo import render_monitoreo_vivo
from .analisis_estadistico import render_analisis_estadistico
from .registros import render_registros
from .dispositivos import render_dispositivos

__all__ = [
    'render_monitoreo_vivo',
    'render_analisis_estadistico', 
    'render_registros',
    'render_dispositivos'
]
